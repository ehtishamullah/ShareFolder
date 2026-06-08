#pragma once

#include <unordered_map>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <future>
#include <functional>
#include <chrono>
#include <iostream>
#include <string>

// ─────────────────────────────────────────────
//  Core data types
// ─────────────────────────────────────────────

using UserID   = std::string;
using SensorID = std::string;

struct Measurement {
    UserID   userID;
    SensorID sensorID;
    float    Sx, Sy;
    float    Tx, Ty;
    double   time;
    float    sigma;
};

// Batch:  userID  →  sensorID  →  [measurements]
using SensorData = std::unordered_map<SensorID, std::vector<Measurement>>;
using Batch      = std::unordered_map<UserID, SensorData>;


// ─────────────────────────────────────────────
//  Double-buffered batch collector
// ─────────────────────────────────────────────

class BatchProcessor {
public:
    // processFn: called once per userID with that user's sensor map
    using ProcessFn = std::function<void(const UserID&, const SensorData&)>;

    explicit BatchProcessor(ProcessFn fn,
                            std::chrono::milliseconds batchDuration =
                                std::chrono::milliseconds(2000))
        : processFn_(std::move(fn))
        , batchDuration_(batchDuration)
        , activeIdx_(0)
        , running_(false)
    {}

    // ── Lifecycle ────────────────────────────

    void start() {
        running_ = true;
        batchStart_ = std::chrono::steady_clock::now();
        timerThread_ = std::thread(&BatchProcessor::timerLoop, this);
    }

    void stop() {
        running_ = false;
        if (timerThread_.joinable())
            timerThread_.join();

        // Flush whatever is left in the active batch
        flushBatch(activeIdx_.load());

        // Wait for any in-flight processing to finish
        drainFutures();
    }

    // ── Ingest (called from your while-loop) ─

    void ingest(const Measurement& m) {
        int idx = activeIdx_.load(std::memory_order_acquire);
        std::lock_guard<std::mutex> lock(batchMutex_[idx]);
        batches_[idx][m.userID][m.sensorID].push_back(m);
    }

    // ─────────────────────────────────────────

private:
    // Two batches + one mutex each
    Batch               batches_[2];
    std::mutex          batchMutex_[2];

    std::atomic<int>    activeIdx_;   // which slot is being written to
    ProcessFn           processFn_;
    std::chrono::milliseconds batchDuration_;

    std::chrono::steady_clock::time_point batchStart_;
    std::thread         timerThread_;
    std::atomic<bool>   running_;

    // Futures of in-flight processing tasks
    std::vector<std::future<void>> futures_;
    std::mutex                     futuresMutex_;

    // ── Timer thread: fires every batchDuration_ ─

    void timerLoop() {
        while (running_) {
            auto deadline = batchStart_ + batchDuration_;
            std::this_thread::sleep_until(deadline);

            if (!running_) break;

            batchStart_ = std::chrono::steady_clock::now();

            // 1. Swap active index (new measurements now go to the other slot)
            int oldIdx = activeIdx_.load(std::memory_order_acquire);
            int newIdx = 1 - oldIdx;

            // 2. Ensure the soon-to-be-active slot is clean before exposing it.
            //    We clear under its own lock so no reader is mid-flight.
            {
                std::lock_guard<std::mutex> lock(batchMutex_[newIdx]);
                batches_[newIdx].clear();
            }

            // 3. Flip — ingest() will now write to newIdx
            activeIdx_.store(newIdx, std::memory_order_release);

            // 4. Process oldIdx in the background (lock while we move data out)
            flushBatch(oldIdx);
        }
    }

    // Launches one async task per userID found in the batch at `idx`
    void flushBatch(int idx) {
        Batch snapshot;
        {
            std::lock_guard<std::mutex> lock(batchMutex_[idx]);
            if (batches_[idx].empty()) return;
            snapshot = std::move(batches_[idx]);   // O(1) move, no copy
            batches_[idx].clear();
        }

        // Fire one task per user — fully parallel
        std::lock_guard<std::mutex> flock(futuresMutex_);

        drainFutures_nolock();   // clean up completed futures first

        for (auto& [uid, sensorMap] : snapshot) {
            // Capture by value so each task owns its slice of data
            futures_.push_back(
                std::async(std::launch::async,
                    [this, uid = uid, sensorMap = std::move(sensorMap)]() mutable {
                        processFn_(uid, sensorMap);
                    }
                )
            );
        }
    }

    // Remove futures that are already done (non-blocking)
    void drainFutures_nolock() {
        futures_.erase(
            std::remove_if(futures_.begin(), futures_.end(),
                [](std::future<void>& f) {
                    return f.wait_for(std::chrono::seconds(0))
                           == std::future_status::ready;
                }),
            futures_.end()
        );
    }

    // Block until ALL processing futures have finished (used on stop())
    void drainFutures() {
        std::lock_guard<std::mutex> flock(futuresMutex_);
        for (auto& f : futures_)
            f.get();
        futures_.clear();
    }
};
