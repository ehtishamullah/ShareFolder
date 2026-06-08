#include "batch_processor.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// ─────────────────────────────────────────────
//  Your processing logic lives here
// ─────────────────────────────────────────────

void processUser(const UserID& uid, const SensorData& sensorMap) {
    std::cout << "[Process] UserID=" << uid << "\n";

    for (const auto& [sid, measurements] : sensorMap) {
        std::cout << "  SensorID=" << sid
                  << "  count=" << measurements.size() << "\n";

        for (const auto& m : measurements) {
            // Access m.Sx, m.Sy, m.Tx, m.Ty, m.sigma, m.time ...
            (void)m;
        }
    }
}

// ─────────────────────────────────────────────
//  Simulated main / while-loop
// ─────────────────────────────────────────────

int main() {
    BatchProcessor processor(processUser,
                             std::chrono::milliseconds(2000));
    processor.start();

    // ── Simulate your data-reception while-loop ─────────────

    auto runUntil = std::chrono::steady_clock::now()
                  + std::chrono::seconds(7);   // run for 7 s

    int counter = 0;
    while (std::chrono::steady_clock::now() < runUntil) {

        // Replace this block with your actual sensor read
        Measurement m;
        m.userID   = "user_" + std::to_string(counter % 3);   // 3 users
        m.sensorID = "sensor_" + std::to_string(counter % 2); // 2 sensors each
        m.Sx    = 1.0f * counter;
        m.Sy    = 2.0f * counter;
        m.Tx    = 3.0f * counter;
        m.Ty    = 4.0f * counter;
        m.time  = counter * 0.01;
        m.sigma = 0.1f;

        processor.ingest(m);

        ++counter;
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 Hz
    }

    // ── Clean shutdown ──────────────────────────────────────
    processor.stop();   // flushes remaining data, waits for all tasks

    std::cout << "Done. Ingested " << counter << " measurements.\n";
    return 0;
}
