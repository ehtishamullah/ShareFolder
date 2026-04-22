clc; clear; close all;

%% PARAMETERS
numSensors = 4;
baseline = 300; % NM total length
offset = 20;    % NM off-axis perturbation

% Sensor positions (collinear + slight offset)
sensorPos = [0     0;
            0 baseline/4;
           0  2*baseline/3;
           offset  baseline/2]; % offset sensor

%% TARGET REGION (grid)
[xg, yg] = meshgrid(-500:20:700, -550:20:550);

bearingNoiseStd = deg2rad(2);
% bearingNoiseMatrix = [sin(bearingNoiseStd), -cos(bearingNoiseStd)];
% Q_noise = inv(bearingNoiseMatrix'*bearingNoiseMatrix);

GDOP_map = zeros(size(xg));

for k = 1:numel(xg)

    target = [xg(k), yg(k)];

    A = [];

    for i = 1:numSensors
        dx = target(1) - sensorPos(i,1);
        dy = target(2) - sensorPos(i,2);
        theta = atan2(dy, dx);

        A = [A;
             sin(theta), -cos(theta)];
    end

    % GDOP-like metric [Add bearing Noise STd or Variance here]
    J = (A'*A);%/ bearingNoiseStd^2;
    Q = inv(J);
    
    GDOP_map(k) = sqrt(trace(Q));

end

%% PLOT GDOP HEATMAP
figure(1);
imagesc(-500:10:700, -550:10:550, ((bearingNoiseStd.*GDOP_map).*1852));
set(gca,'YDir','normal', 'ColorScale', 'log');
c = colorbar;
title('Positional Inaccuracy Map');
xlabel('Range (NM)');
ylabel('Cross-range (NM)');
c.Label.String = 'RMS Positional Error (meter)';

hold on;
plot(sensorPos(:,1), sensorPos(:,2),'wo','MarkerFaceColor','k');

figure(2);
contourf(xg, yg, ((bearingNoiseStd.*GDOP_map).*1852), [100, 200, 400, 500, 1000, 1500, 2000, 2500], "ShowText",true,...
    "LabelFormat","%0.1f m", "FaceAlpha",0.25);
set(gca,'YDir','normal', 'ColorScale', 'log');
title('Positional Inaccuracy Map');
xlabel('Range (NM)');
ylabel('Cross-range (NM)');