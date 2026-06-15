function motor_gui_sim_with_risk
clc;

oldTimers = timerfindall('Tag','MotorSimOnly2SegTimerRisk');
for k = 1:numel(oldTimers)
    try
        stop(oldTimers(k));
        delete(oldTimers(k));
    catch
    end
end

ids = [1, 2, 3, 4, 5, 6];
home_angles = [0, 0, 0, 0, 0, 0];

% ==================== UI color palette ====================
UI.bg        = [0.955 0.965 0.980];   % main background
UI.cardBg    = [0.985 0.990 1.000];   % edit-box background
UI.navy      = [0.055 0.145 0.290];   % main dark blue
UI.border    = [0.220 0.330 0.480];   % border color
UI.text      = [0.070 0.090 0.120];   % text color

hFig = figure('Name','Continuum Robot Control Interface - Drive-Curvature Consistency Simulation', ...
    'NumberTitle','off', ...
    'Position',[220 90 1380 760], ...
    'MenuBar','none', ...
    'ToolBar','none', ...
    'Resize','off', ...
    'Color',UI.bg, ...
    'WindowKeyPressFcn', @onKeyDown, ...
    'WindowKeyReleaseFcn', @onKeyUp, ...
    'WindowButtonUpFcn', @onMouseUp, ...
    'CloseRequestFcn', @onClose);

S.key.up    = false;
S.key.down  = false;
S.key.left  = false;
S.key.right = false;
S.key.space = false;

S.current_angles = home_angles;
S.home_angles    = home_angles;
S.ids = ids;

S.rateHz = 50;
S.speed  = 50;
S.accel  = 300;
S.isSync = 0;

% ============================================================
% Directional velocity vectors for the six motors
% IMPORTANT:
% Down is the opposite of Up.
% Left is the opposite of Right.
% If the physical direction is reversed after testing, exchange the labels
% of the corresponding pair or multiply the corresponding vector by -1.
% ============================================================
S.v_up    = [60, 45,  0,  0,  0, 45];
S.v_down  = -S.v_up;

S.v_right = [ 0, 45, 30, 30,  0,  0];
S.v_left  = -S.v_right;

S.vel = zeros(1,6);
S.reset.active  = false;
S.reset.vmax    = 360 * ones(1,6);
S.reset.accel   = 80;
S.reset.doneTol = 0.05;
S.angleLimit = 360;

S.transRatio = 1.0;        % l_i = transRatio * theta_i

% Stiffness risk thresholds
S.risk.th1 = 45;           % First threshold: start attenuating angular increments
S.risk.th2 = 90;           % Second threshold: block risk-increasing angular increments

% Consistency error threshold
S.err.th = 8.0;            % mm

% Initialize displayed quantities
S.R1 = 0;
S.R2 = 0;
S.deltaRaw = zeros(1,6);
S.deltaCorr = zeros(1,6);
S.E = 0;
S.safetyState = 'Normal';

% ==================== Left control panel ====================
uicontrol('Parent',hFig, ...
    'Style','text', ...
    'String','Control Panel', ...
    'Units','normalized', ...
    'Position',[0.04 0.91 0.38 0.05], ...
    'FontSize',20, ...
    'FontWeight','bold', ...
    'ForegroundColor',UI.navy, ...
    'BackgroundColor',UI.bg);

S.hState = uicontrol('Parent',hFig, ...
    'Style','text', ...
    'String','Status: Standby', ...
    'Units','normalized', ...
    'Position',[0.04 0.85 0.40 0.04], ...
    'FontSize',13, ...
    'FontWeight','bold', ...
    'ForegroundColor',UI.text, ...
    'HorizontalAlignment','left', ...
    'BackgroundColor',UI.bg);

S.hAngle = uicontrol('Parent',hFig, ...
    'Style','edit', ...
    'String',sprintf(['Current angles theta:\n' ...
                      '[0.0  0.0  0.0]\n' ...
                      '[0.0  0.0  0.0]']), ...
    'Units','normalized', ...
    'Position',[0.04 0.71 0.40 0.12], ...
    'FontSize',12, ...
    'HorizontalAlignment','left', ...
    'Enable','inactive', ...
    'Max',2, ...
    'BackgroundColor',UI.cardBg, ...
    'ForegroundColor',UI.text);

S.hRisk = uicontrol('Parent',hFig, ...
    'Style','edit', ...
    'String',sprintf(['Stiffness risk indicators:\n' ...
                      'R1 = 0.00   R2 = 0.00\n' ...
                      'Thresholds: T1 = %.1f, T2 = %.1f'], S.risk.th1, S.risk.th2), ...
    'Units','normalized', ...
    'Position',[0.04 0.59 0.40 0.105], ...
    'FontSize',12, ...
    'HorizontalAlignment','left', ...
    'Enable','inactive', ...
    'Max',2, ...
    'BackgroundColor',UI.cardBg, ...
    'ForegroundColor',UI.text);

S.hDelta = uicontrol('Parent',hFig, ...
    'Style','edit', ...
    'String',sprintf(['Corrected angular increments dTheta'':\n' ...
                      '[0.000  0.000  0.000]\n' ...
                      '[0.000  0.000  0.000]']), ...
    'Units','normalized', ...
    'Position',[0.04 0.46 0.40 0.12], ...
    'FontSize',12, ...
    'HorizontalAlignment','left', ...
    'Enable','inactive', ...
    'Max',2, ...
    'BackgroundColor',UI.cardBg, ...
    'ForegroundColor',UI.text);

S.hError = uicontrol('Parent',hFig, ...
    'Style','edit', ...
    'String',sprintf(['Consistency error:\n' ...
                      'E = 0.000 mm\n' ...
                      'Safety state: Normal']), ...
    'Units','normalized', ...
    'Position',[0.04 0.34 0.40 0.105], ...
    'FontSize',12, ...
    'HorizontalAlignment','left', ...
    'Enable','inactive', ...
    'Max',2, ...
    'BackgroundColor',UI.cardBg, ...
    'ForegroundColor',UI.text);

% ==================== Direction and function buttons ====================
makeHoldButton(hFig, 'Up',    [0.185 0.225 0.135 0.075], 'up');
makeHoldButton(hFig, 'Left',  [0.070 0.130 0.135 0.075], 'left');
makeHoldButton(hFig, 'Right', [0.295 0.130 0.135 0.075], 'right');
makeHoldButton(hFig, 'Down',  [0.185 0.035 0.135 0.075], 'down');

makeClickButton(hFig, 'Reset', [0.055 0.235 0.095 0.055], ...
    @(~,~)triggerReset(hFig), 'reset');

makeClickButton(hFig, 'Stop', [0.350 0.235 0.080 0.055], ...
    @(~,~)stopMotion(hFig), 'stop');

uicontrol('Parent',hFig, ...
    'Style','text', ...
    'String','Hold a direction button for continuous motion; dTheta is corrected according to R1 and R2.', ...
    'Units','normalized', ...
    'Position',[0.04 0.005 0.40 0.025], ...
    'FontSize',11, ...
    'HorizontalAlignment','center', ...
    'ForegroundColor',UI.border, ...
    'BackgroundColor',UI.bg);

% ==================== Right simulation area ====================
uicontrol('Parent',hFig, ...
    'Style','text', ...
    'String','Two-Segment Continuum Robot Simulation', ...
    'Units','normalized', ...
    'Position',[0.54 0.91 0.36 0.05], ...
    'FontSize',20, ...
    'FontWeight','bold', ...
    'ForegroundColor',UI.navy, ...
    'BackgroundColor',UI.bg);

S.hAxSim = axes('Parent',hFig, ...
    'Units','normalized', ...
    'Position',[0.50 0.11 0.45 0.76]);

S = initSimPlot(S);

% Initial calculation and display
[S.R1, S.R2] = computeRisk(S, S.current_angles);

set(hFig, 'UserData', S);
updateSimPlot(hFig);
updateStatus(hFig, S, 'Status: Standby');

% ==================== Timer ====================
t = timer('ExecutionMode','fixedSpacing', ...
    'Period', 1 / S.rateHz, ...
    'TimerFcn', @(~,~)onTick(hFig), ...
    'Tag','MotorSimOnly2SegTimerRisk');

start(t);

end

function makeHoldButton(hFig, label, pos, direction)

buttonFace = [0.925 0.950 0.980];
buttonEdge = [0.055 0.145 0.290];
buttonText = [0.055 0.145 0.290];
shadowColor = [0.78 0.82 0.88];

ax = axes('Parent',hFig, ...
    'Units','normalized', ...
    'Position',pos, ...
    'XLim',[0 1], ...
    'YLim',[0 1], ...
    'XTick',[], ...
    'YTick',[], ...
    'Color','none', ...
    'Visible','off');

hold(ax,'on');

% Shadow layer
shadow = rectangle(ax, ...
    'Position',[0.045 0.030 0.910 0.860], ...
    'Curvature',0.16, ...
    'FaceColor',shadowColor, ...
    'EdgeColor','none');

% Main button
btn = rectangle(ax, ...
    'Position',[0.020 0.080 0.910 0.860], ...
    'Curvature',0.16, ...
    'FaceColor',buttonFace, ...
    'EdgeColor',buttonEdge, ...
    'LineWidth',1.8);

txt = text(ax,0.475,0.510,label, ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'FontSize',17, ...
    'FontWeight','bold', ...
    'Color',buttonText);

set([ax, shadow, btn, txt], ...
    'HitTest','on', ...
    'ButtonDownFcn', @(~,~)onButtonDown(hFig, direction));

end

function makeClickButton(hFig, label, pos, callbackFcn, type)

switch type
    case 'reset'
        buttonFace = [0.925 0.950 0.980];
        buttonEdge = [0.055 0.145 0.290];
        buttonText = [0.055 0.145 0.290];

    case 'stop'
        buttonFace = [0.975 0.925 0.915];
        buttonEdge = [0.550 0.120 0.090];
        buttonText = [0.550 0.120 0.090];

    otherwise
        buttonFace = [0.925 0.950 0.980];
        buttonEdge = [0.055 0.145 0.290];
        buttonText = [0.055 0.145 0.290];
end

shadowColor = [0.78 0.82 0.88];

ax = axes('Parent',hFig, ...
    'Units','normalized', ...
    'Position',pos, ...
    'XLim',[0 1], ...
    'YLim',[0 1], ...
    'XTick',[], ...
    'YTick',[], ...
    'Color','none', ...
    'Visible','off');

hold(ax,'on');

% Shadow layer
shadow = rectangle(ax, ...
    'Position',[0.045 0.030 0.910 0.860], ...
    'Curvature',0.16, ...
    'FaceColor',shadowColor, ...
    'EdgeColor','none');

% Main button
btn = rectangle(ax, ...
    'Position',[0.020 0.080 0.910 0.860], ...
    'Curvature',0.16, ...
    'FaceColor',buttonFace, ...
    'EdgeColor',buttonEdge, ...
    'LineWidth',1.6);

txt = text(ax,0.475,0.510,label, ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'FontSize',12.5, ...
    'FontWeight','bold', ...
    'Color',buttonText);

set([ax, shadow, btn, txt], ...
    'HitTest','on', ...
    'ButtonDownFcn', callbackFcn);

end

function onButtonDown(hFig, direction)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end
S.reset.active = false;

switch direction
    case 'up'
        S.key.up = true;
        S.key.down = false;

    case 'down'
        S.key.down = true;
        S.key.up = false;

    case 'left'
        S.key.left = true;
        S.key.right = false;

    case 'right'
        S.key.right = true;
        S.key.left = false;
end

set(hFig, 'UserData', S);
updateStatus(hFig, S, ['Status: Moving ', direction]);

end

function onMouseUp(hFig, ~)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end

S.key.up    = false;
S.key.down  = false;
S.key.left  = false;
S.key.right = false;

set(hFig, 'UserData', S);
updateStatus(hFig, S, 'Status: Direction button released');

end

function onTick(hFig)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end

dt = 1 / S.rateHz;

if S.key.space
    S.reset.active = true;
    S.key.space = false;
    S.vel(:) = 0;
    S.deltaRaw(:) = 0;
    S.deltaCorr(:) = 0;
    set(hFig, 'UserData', S);
    updateStatus(hFig, S, 'Status: Starting flexible reset');
    return;
end

% ==================== Piecewise reset ====================
if S.reset.active

    deltaToHome = S.home_angles - S.current_angles;

    if all(abs(deltaToHome) <= S.reset.doneTol)
        S.current_angles = S.home_angles;
        S.vel(:) = 0;
        S.reset.active = false;
        S.deltaRaw(:) = 0;
        S.deltaCorr(:) = 0;
        S.E = 0;
        [S.R1, S.R2] = computeRisk(S, S.current_angles);
        S.safetyState = 'Reset completed';

        set(hFig, 'UserData', S);
        updateSimPlot(hFig);
        updateStatus(hFig, S, 'Status: Reset completed');
        return;
    end

    [R1now, R2now] = computeRisk(S, S.current_angles);
    weight = ones(1,6);
    if R1now > R2now
        weight(1:3) = 1.20;
        weight(4:6) = 0.75;
        S.safetyState = 'Reset: releasing Segment 1 first';
    elseif R2now > R1now
        weight(1:3) = 0.75;
        weight(4:6) = 1.20;
        S.safetyState = 'Reset: releasing Segment 2 first';
    else
        S.safetyState = 'Reset: releasing both segments';
    end

    v_des = sign(deltaToHome) .* min(abs(deltaToHome) / dt, S.reset.vmax);
    v_des = v_des .* weight;

    dv = v_des - S.vel;
    lim = S.reset.accel * dt;
    S.vel = S.vel + max(min(dv, lim), -lim);

    stepRaw = S.vel * dt;

    over = abs(stepRaw) > abs(deltaToHome);
    stepRaw(over) = deltaToHome(over);

    [stepCorr, S] = applyRiskConstraint(S, stepRaw);

    S.current_angles = S.current_angles + stepCorr;
    S.current_angles = max(min(S.current_angles, S.angleLimit), -S.angleLimit);

    set(hFig, 'UserData', S);
    updateSimPlot(hFig);
    updateStatus(hFig, S, 'Status: Piecewise-curvature reset in progress');
    return;
end

% ==================== Normal direction control ====================
v_des = [0 0 0 0 0 0];

if S.key.up
    v_des = v_des + S.v_up;
end

if S.key.down
    v_des = v_des + S.v_down;
end

if S.key.left
    v_des = v_des + S.v_left;
end

if S.key.right
    v_des = v_des + S.v_right;
end

dv = v_des - S.vel;
lim = S.accel * dt;
S.vel = S.vel + max(min(dv, lim), -lim);

stepRaw = S.vel * dt;
S.deltaRaw = stepRaw;

if any(abs(stepRaw) > 1e-6)

    [stepCorr, S] = applyRiskConstraint(S, stepRaw);

    S.current_angles = S.current_angles + stepCorr;
    S.current_angles = max(min(S.current_angles, S.angleLimit), -S.angleLimit);

    set(hFig, 'UserData', S);
    updateSimPlot(hFig);
    updateStatus(hFig, S, ['Status: Simulating motion (', S.safetyState, ')']);

else
    S.deltaRaw(:) = 0;
    S.deltaCorr(:) = 0;
    [S.R1, S.R2] = computeRisk(S, S.current_angles);
    S.E = 0;
    if ~S.reset.active
        S.safetyState = 'Normal';
    end
    set(hFig, 'UserData', S);
    updateStatus(hFig, S, 'Status: Standby');
end

end

function [deltaCorr, S] = applyRiskConstraint(S, deltaRaw)

qNow = S.current_angles;
qRaw = qNow + deltaRaw;

[R1now, R2now] = computeRisk(S, qNow);
[R1raw, R2raw] = computeRisk(S, qRaw);

deltaCorr = deltaRaw;
factor1 = riskFactor(R1now, R1raw, S.risk.th1, S.risk.th2);
factor2 = riskFactor(R2now, R2raw, S.risk.th1, S.risk.th2);

deltaCorr(1:3) = deltaRaw(1:3) * factor1;
deltaCorr(4:6) = deltaRaw(4:6) * factor2;

qCorr = qNow + deltaCorr;
[S.R1, S.R2] = computeRisk(S, qCorr);

S.E = computeConsistencyError(qNow, deltaRaw, deltaCorr);

S.deltaRaw = deltaRaw;
S.deltaCorr = deltaCorr;

if factor1 == 0 || factor2 == 0
    S.safetyState = 'High risk: risk-increasing motion blocked';
elseif factor1 < 1 || factor2 < 1
    S.safetyState = 'Medium risk: angular increment attenuated';
elseif S.E > S.err.th
    S.safetyState = 'Large consistency error';
else
    S.safetyState = 'Normal';
end

end

function f = riskFactor(Rnow, Rnext, th1, th2)

if Rnext <= Rnow
    f = 1.0;
    return;
end

if Rnext < th1
    f = 1.0;

elseif Rnext < th2
    ratio = (Rnext - th1) / (th2 - th1);
    f = max(0.30, 1.0 - 0.70 * ratio);

else
    f = 0.0;
end

end

function [R1, R2] = computeRisk(S, q)

L = S.transRatio * q;

seg1 = L(1:3);
seg2 = L(4:6);

R1 = std(seg1, 1);
R2 = std(seg2, 1);

end

function E = computeConsistencyError(qNow, deltaRaw, deltaCorr)

[~, ~, ~, ~, tipRaw] = calcContinuumShape(qNow + deltaRaw);
[~, ~, ~, ~, tipCorr] = calcContinuumShape(qNow + deltaCorr);
E = norm(tipRaw - tipCorr);

end

function onKeyDown(hFig, evt)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end

S.reset.active = false;

switch evt.Key
    case 'uparrow'
        S.key.up = true;
        S.key.down = false;

    case 'downarrow'
        S.key.down = true;
        S.key.up = false;

    case 'leftarrow'
        S.key.left = true;
        S.key.right = false;

    case 'rightarrow'
        S.key.right = true;
        S.key.left = false;

    case 'space'
        S.key.space = true;
end

set(hFig, 'UserData', S);
updateStatus(hFig, S, ['Status: Keyboard ', evt.Key]);

end

function onKeyUp(hFig, evt)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end

switch evt.Key
    case 'uparrow'
        S.key.up = false;

    case 'downarrow'
        S.key.down = false;

    case 'leftarrow'
        S.key.left = false;

    case 'rightarrow'
        S.key.right = false;

    case 'space'
        S.key.space = false;
end

set(hFig, 'UserData', S);
updateStatus(hFig, S, 'Status: Key released');

end

function triggerReset(hFig)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end

S.key.up    = false;
S.key.down  = false;
S.key.left  = false;
S.key.right = false;
S.key.space = false;

S.vel(:) = 0;
S.reset.active = true;

set(hFig, 'UserData', S);
updateStatus(hFig, S, 'Status: Starting piecewise-curvature reset');

end

function stopMotion(hFig)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end

S.key.up    = false;
S.key.down  = false;
S.key.left  = false;
S.key.right = false;
S.key.space = false;

S.vel(:) = 0;
S.reset.active = false;
S.deltaRaw(:) = 0;
S.deltaCorr(:) = 0;
S.E = 0;
S.safetyState = 'Stopped';

set(hFig, 'UserData', S);
updateStatus(hFig, S, 'Status: Stopped');

end

function S = initSimPlot(S)

ax = S.hAxSim;
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 35, 25);

set(ax, ...
    'FontName','Arial', ...
    'FontSize',14, ...
    'LineWidth',1.2, ...
    'Color',[0.985 0.990 1.000], ...
    'XColor',[0.12 0.16 0.22], ...
    'YColor',[0.12 0.16 0.22], ...
    'ZColor',[0.12 0.16 0.22], ...
    'GridColor',[0.72 0.78 0.86], ...
    'GridAlpha',0.35);

xlabel(ax, 'X / mm', 'FontName','Arial', 'FontSize',16, 'FontWeight','bold');
ylabel(ax, 'Y / mm', 'FontName','Arial', 'FontSize',16, 'FontWeight','bold');
zlabel(ax, 'Z / mm', 'FontName','Arial', 'FontSize',16, 'FontWeight','bold');

xlim(ax, [-60 60]);
ylim(ax, [-60 60]);
zlim(ax, [0 90]);

title(ax, 'Tip Bending Simulation of the Two-Segment Continuum Robot', ...
    'FontName','Arial', ...
    'FontSize',20, ...
    'FontWeight','bold', ...
    'Color',[0.055 0.145 0.290]);

[pts1, pts2, ptsAll, jointPoint, tip] = calcContinuumShape(S.current_angles);

S.hBase = plot3(ax, 0, 0, 0, 'ks', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', 'k');

S.hSeg1 = plot3(ax, pts1(:,1), pts1(:,2), pts1(:,3), ...
    'Color', [0.00 0.45 0.74], ...
    'LineWidth', 5);

S.hSeg2 = plot3(ax, pts2(:,1), pts2(:,2), pts2(:,3), ...
    'Color', [0.85 0.33 0.10], ...
    'LineWidth', 5);

S.hJoint = plot3(ax, jointPoint(1), jointPoint(2), jointPoint(3), ...
    'ko', ...
    'MarkerSize', 7, ...
    'MarkerFaceColor', 'k');

S.hTip = plot3(ax, tip(1), tip(2), tip(3), ...
    'ro', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', 'r');

S.hShadow = plot3(ax, ptsAll(:,1), ptsAll(:,2), zeros(size(ptsAll,1),1), ...
    '--', ...
    'Color', [0.70 0.70 0.70], ...
    'LineWidth', 1.2);

legend(ax, ...
    [S.hBase, S.hSeg1, S.hSeg2, S.hJoint, S.hTip, S.hShadow], ...
    {'Base', 'Segment 1 (40 mm)', 'Segment 2 (40 mm)', 'Segment connection', 'Tip point', 'Ground projection'}, ...
    'Location','northeastoutside', ...
    'FontName','Arial', ...
    'FontSize',12);

end

function updateSimPlot(hFig)

if ~isvalid(hFig), return; end

S = get(hFig, 'UserData');
if isempty(S), return; end

[pts1, pts2, ptsAll, jointPoint, tip] = calcContinuumShape(S.current_angles);

if isfield(S, 'hSeg1') && isvalid(S.hSeg1)
    set(S.hSeg1, ...
        'XData', pts1(:,1), ...
        'YData', pts1(:,2), ...
        'ZData', pts1(:,3));
end

if isfield(S, 'hSeg2') && isvalid(S.hSeg2)
    set(S.hSeg2, ...
        'XData', pts2(:,1), ...
        'YData', pts2(:,2), ...
        'ZData', pts2(:,3));
end

if isfield(S, 'hJoint') && isvalid(S.hJoint)
    set(S.hJoint, ...
        'XData', jointPoint(1), ...
        'YData', jointPoint(2), ...
        'ZData', jointPoint(3));
end

if isfield(S, 'hTip') && isvalid(S.hTip)
    set(S.hTip, ...
        'XData', tip(1), ...
        'YData', tip(2), ...
        'ZData', tip(3));
end

if isfield(S, 'hShadow') && isvalid(S.hShadow)
    set(S.hShadow, ...
        'XData', ptsAll(:,1), ...
        'YData', ptsAll(:,2), ...
        'ZData', zeros(size(ptsAll,1),1));
end

drawnow limitrate nocallbacks;

end

function [pts1, pts2, ptsAll, jointPoint, tip] = calcContinuumShape(q)

L1 = 40;
L2 = 40;
N1 = 30;
N2 = 30;

bendX1 = 0.0020 * ( q(2) + q(3) - q(1) );
bendY1 = 0.0020 * ( q(1) + q(2) - q(3) );
bendX2 = 0.0020 * ( q(4) + q(5) - q(6) );
bendY2 = 0.0020 * ( q(6) + q(5) - q(4) );

maxBend = 1.2;
bendX1 = max(min(bendX1, maxBend), -maxBend);
bendY1 = max(min(bendY1, maxBend), -maxBend);
bendX2 = max(min(bendX2, maxBend), -maxBend);
bendY2 = max(min(bendY2, maxBend), -maxBend);

p0 = [0; 0; 0];
R0 = eye(3);

[pts1, pJoint, RJoint] = integrateSegment(p0, R0, L1, N1, bendX1, bendY1);
[pts2, pTip, ~] = integrateSegment(pJoint, RJoint, L2, N2, bendX2, bendY2);

ptsAll = [pts1; pts2(2:end,:)];

jointPoint = pJoint';
tip = pTip';

end

function [pts, pEnd, REnd] = integrateSegment(pStart, RStart, L, N, bendX, bendY)

ds = L / N;

pts = zeros(N + 1, 3);
p = pStart;
R = RStart;

pts(1,:) = p';

for i = 1:N
    dRx = bendY / N;
    dRy = bendX / N;
    dR = rotY(dRy) * rotX(dRx);
    R = R * dR;
    p = p + R * [0; 0; ds];
    pts(i + 1,:) = p';
end

pEnd = p;
REnd = R;

end

function R = rotX(a)

R = [1 0 0;
     0 cos(a) -sin(a);
     0 sin(a)  cos(a)];

end

function R = rotY(a)

R = [ cos(a) 0 sin(a);
      0      1 0;
     -sin(a) 0 cos(a)];

end

function updateStatus(hFig, S, msg)

if ~isvalid(hFig), return; end

if isfield(S, 'hState') && isvalid(S.hState)
    set(S.hState, 'String', msg);
end

if isfield(S, 'hAngle') && isvalid(S.hAngle)
    angleText = sprintf(['Current angles theta:\n' ...
        '[%.1f  %.1f  %.1f]\n' ...
        '[%.1f  %.1f  %.1f]'], ...
        S.current_angles(1), S.current_angles(2), S.current_angles(3), ...
        S.current_angles(4), S.current_angles(5), S.current_angles(6));

    set(S.hAngle, 'String', angleText);
end

if isfield(S, 'hRisk') && isvalid(S.hRisk)
    riskText = sprintf(['Stiffness risk indicators:\n' ...
        'R1 = %.2f   R2 = %.2f\n' ...
        'Thresholds: T1 = %.1f, T2 = %.1f'], ...
        S.R1, S.R2, S.risk.th1, S.risk.th2);

    set(S.hRisk, 'String', riskText);
end

if isfield(S, 'hDelta') && isvalid(S.hDelta)
    deltaText = sprintf(['Corrected angular increments dTheta'':\n' ...
        '[%.3f  %.3f  %.3f]\n' ...
        '[%.3f  %.3f  %.3f]'], ...
        S.deltaCorr(1), S.deltaCorr(2), S.deltaCorr(3), ...
        S.deltaCorr(4), S.deltaCorr(5), S.deltaCorr(6));

    set(S.hDelta, 'String', deltaText);
end

if isfield(S, 'hError') && isvalid(S.hError)
    errText = sprintf(['Consistency error:\n' ...
        'E = %.3f mm\n' ...
        'Safety state: %s'], ...
        S.E, S.safetyState);

    set(S.hError, 'String', errText);
end

end

function onClose(hFig, ~)

timers = timerfindall('Tag','MotorSimOnly2SegTimerRisk');

for k = 1:numel(timers)
    try
        stop(timers(k));
        delete(timers(k));
    catch
    end
end

if isvalid(hFig)
    delete(hFig);
end

end