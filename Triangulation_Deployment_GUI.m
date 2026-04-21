function triangulation_planner()

clc; close all;

%% DEFAULT PARAMETERS
params.N = 4;
params.baseline = 200;     % km
params.offset = 10;        % km
params.R = 250;            % target range
params.sigma = deg2rad(1); % bearing noise

%% CREATE FIGURE
f = figure('Name','Triangulation Deployment Planner',...
           'Position',[100 100 1000 600]);

ax = axes('Parent',f,'Position',[0.35 0.1 0.6 0.8]);

%% UI CONTROLS

uicontrol(f,'Style','text','Position',[20 520 150 20],'String','# Sensors');
sN = uicontrol(f,'Style','slider','Min',3,'Max',6,'Value',4,...
    'Position',[20 500 200 20],'Callback',@updatePlot);

uicontrol(f,'Style','text','Position',[20 460 150 20],'String','Baseline (km)');
sB = uicontrol(f,'Style','slider','Min',50,'Max',400,'Value',200,...
    'Position',[20 440 200 20],'Callback',@updatePlot);

uicontrol(f,'Style','text','Position',[20 400 150 20],'String','Offset (km)');
sO = uicontrol(f,'Style','slider','Min',0,'Max',50,'Value',10,...
    'Position',[20 380 200 20],'Callback',@updatePlot);

uicontrol(f,'Style','text','Position',[20 340 150 20],'String','Target Range (km)');
sR = uicontrol(f,'Style','slider','Min',100,'Max',400,'Value',250,...
    'Position',[20 320 200 20],'Callback',@updatePlot);

uicontrol(f,'Style','text','Position',[20 280 150 20],'String','Bearing Noise (deg)');
sS = uicontrol(f,'Style','slider','Min',0.1,'Max',5,'Value',1,...
    'Position',[20 260 200 20],'Callback',@updatePlot);

%% INITIAL PLOT
updatePlot();

%% CALLBACK FUNCTION
function updatePlot(~,~)

    % Read UI values
    N = round(get(sN,'Value'));
    baseline = get(sB,'Value');
    offset = get(sO,'Value');
    R = get(sR,'Value');
    sigma = deg2rad(get(sS,'Value'));

    % Generate sensor positions
    x = linspace(-baseline/2, baseline/2, N);
    y = zeros(1,N);

    % Add offset sensor (last one)
    if N >= 4
        y(end) = offset;
    end

    sensorPos = [x(:), y(:)];

    % Target grid
    [X,Y] = meshgrid(-100:20:100, R-100:20:R+100);

    CRLB_map = zeros(size(X));

    % Compute CRLB over grid
    for k = 1:numel(X)

        xt = X(k);
        yt = Y(k);

        J = zeros(2,2);

        for i = 1:N
            dx = xt - sensorPos(i,1);
            dy = yt - sensorPos(i,2);
            r2 = dx^2 + dy^2;

            H = [-dy/r2 , dx/r2];
            J = J + (H' * H);
        end

        J = J / sigma^2;

        if rcond(J) < 1e-10
            CRLB_map(k) = NaN;
        else
            C = inv(J);
            CRLB_map(k) = trace(C);
        end
    end

    %% PLOT
    axes(ax);
    cla; hold on; grid on;

    % Heatmap
    imagesc(X(1,:), Y(:,1), log10(CRLB_map));
    set(gca,'YDir','normal');
    colormap jet;
    colorbar;
    title('log_{10}(CRLB) - Lower is Better');

    % Sensors
    plot(sensorPos(:,1), sensorPos(:,2),'wo','MarkerFaceColor','k','MarkerSize',8);

    % Draw LoBs toward center target
    for i = 1:N
        dx = 0 - sensorPos(i,1);
        dy = R - sensorPos(i,2);
        theta = atan2(dy, dx);

        t = linspace(0,400,100);
        plot(sensorPos(i,1)+t*cos(theta), sensorPos(i,2)+t*sin(theta),'w--');
    end

    xlabel('Cross-range (km)');
    ylabel('Range (km)');
    axis equal;

end

end