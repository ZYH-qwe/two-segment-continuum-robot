

clc; clear; close all;

fontName   = 'Arial';
fontAxis   = 16;
fontLabel  = 18;
fontTitle  = 20;
fontLegend = 14;
fontText   = 15;
lineWidth  = 3.5;    

N = 120;                 
dt = 0.02;               

eta = 1.0;               
D0 = 45;                  
Mth1 = 0.70;             
Mth2 = 1.20;              


theta0 = [60, 10, 0, 0, 0, 0];


deltaTheta = [2, 0, 0, 0, 0, 0];


theta_raw = theta0;       % Angles under unconstrained control
theta_con = theta0;       % Angles under risk-constrained control

R1_raw = zeros(N,1);
R2_raw = zeros(N,1);
R1_con = zeros(N,1);
R2_con = zeros(N,1);

gamma1_log = zeros(N,1);
gamma2_log = zeros(N,1);

deltaRaw_log  = zeros(N,6);   % Initial angular increment Δθ
deltaCorr_log = zeros(N,6);   % Corrected angular increment Δθ'
thetaCon_log = zeros(N,6);    % Angles under constrained control
thetaRaw_log = zeros(N,6);    % Angles under unconstrained control
tipRaw_log   = zeros(N,3);    % Tip positions under unconstrained control
tipCon_log   = zeros(N,3);    % Tip positions under constrained control


for k = 1:N

 
    theta_raw = theta_raw + deltaTheta;

    [R1_raw(k), R2_raw(k)] = calcRisk(theta_raw, eta, D0);
    thetaRaw_log(k,:) = theta_raw;
    tipRaw_log(k,:) = calcTipPosition(theta_raw, eta);


    [deltaThetaCorr, gamma1, gamma2] = correctDeltaTheta( ...
        theta_con, deltaTheta, eta, D0, Mth1, Mth2);


    deltaRaw_log(k,:)  = deltaTheta;
    deltaCorr_log(k,:) = deltaThetaCorr;


    theta_con = theta_con + deltaThetaCorr;

    [R1_con(k), R2_con(k)] = calcRisk(theta_con, eta, D0);
    thetaCon_log(k,:) = theta_con;
    tipCon_log(k,:) = calcTipPosition(theta_con, eta);

    gamma1_log(k) = gamma1;
    gamma2_log(k) = gamma2;
end



t = (1:N) * dt;

Rmax_raw = max(R1_raw, R2_raw);
Rmax_con = max(R1_con, R2_con);



figure('Color','w','Position',[120 80 1250 850]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');


nexttile;
plot(t, R1_raw, 'LineWidth', lineWidth); hold on;
plot(t, R2_raw, 'LineWidth', lineWidth);
plot([t(1), t(end)], [Mth1, Mth1], '--', 'LineWidth', 1.6);
plot([t(1), t(end)], [Mth2, Mth2], '--', 'LineWidth', 1.6);

grid on;
box on;

xlabel('Time / s');
ylabel('Risk indicator');
title('(a) Unconstrained segment risks');

legend({'R_1','R_2','R_{th1}','R_{th2}'}, ...
    'Location','northwest');

nexttile;
plot(t, R1_con, 'LineWidth', lineWidth); hold on;
plot(t, R2_con, 'LineWidth', lineWidth);
plot([t(1), t(end)], [Mth1, Mth1], '--', 'LineWidth', 1.6);
plot([t(1), t(end)], [Mth2, Mth2], '--', 'LineWidth', 1.6);

grid on;
box on;

xlabel('Time / s');
ylabel('Risk indicator');
title('(b) Risk-constrained segment risks');

legend({'R_1','R_2','R_{th1}','R_{th2}'}, ...
    'Location','northwest');

nexttile;
plot(t, Rmax_raw, 'LineWidth', lineWidth); hold on;
plot(t, Rmax_con, 'LineWidth', lineWidth);
plot([t(1), t(end)], [Mth1, Mth1], '--', 'LineWidth', 1.6);
plot([t(1), t(end)], [Mth2, Mth2], '--', 'LineWidth', 1.6);

grid on;
box on;

xlabel('Time / s');
ylabel('R_{max}');
title('(c) Maximum risk indicator');

legend({'Unconstrained','Risk-constrained','R_{th1}','R_{th2}'}, ...
    'Location','northwest');

nexttile;
barData = [max(Rmax_raw), max(Rmax_con); ...
           Rmax_raw(end), Rmax_con(end)];

b = bar(barData);
applyLightBarColors(b);

grid on;
box on;

set(gca,'XTickLabel',{'Maximum','Final'});
ylabel('R_{max}');
title('(d) Summary of R_{max}');

legend({'Original','Constrained'}, ...
    'Location','northwest');

set(findall(gcf,'-property','FontName'),'FontName',fontName);
set(findall(gcf,'-property','FontSize'),'FontSize',fontAxis);
set(findall(gcf,'Type','axes'),'LineWidth',1.2);

print(gcf, 'Fig10_composite_risk_indicator.png', '-dpng', '-r300');

disp('Fig. 10 generated: Fig10_composite_risk_indicator.png');



selectedSteps = [1, 11, 61, 120];


deltaTheta_demo = [2, -1, 1, 0.8, -0.4, 0.4];

gamma1_demo_log = zeros(N,1);
gamma2_demo_log = zeros(N,1);

for k = 1:N
    theta_demo = thetaCon_log(k,:);
    [~, gamma1_demo_log(k), gamma2_demo_log(k)] = correctDeltaTheta( ...
        theta_demo, deltaTheta_demo, eta, D0, Mth1, Mth2);
end

figure('Color','w','Position',[60 80 1550 850]);
tiledlayout(3,2,'Padding','compact','TileSpacing','compact');


nexttile([1 2]);
h1 = plot(1:N, gamma1_demo_log, 'LineWidth', lineWidth); hold on;
h2 = plot(1:N, gamma2_demo_log, 'LineWidth', lineWidth);


p1 = plot(selectedSteps, gamma1_demo_log(selectedSteps), 'o', ...
    'MarkerSize', 8, 'LineWidth', 1.8, ...
    'Color', h1.Color, 'MarkerFaceColor', 'w');
p2 = plot(selectedSteps, gamma2_demo_log(selectedSteps), 's', ...
    'MarkerSize', 8, 'LineWidth', 1.8, ...
    'Color', h2.Color, 'MarkerFaceColor', 'w');

grid on;
box on;
xlabel('Control step');
ylabel('Correction coefficient \gamma');
title('(a) Overall correction trend');
legend([h1 h2 p1 p2], ...
    {'\gamma_1','\gamma_2','Selected steps on \gamma_1','Selected steps on \gamma_2'}, ...
    'Location','northeast', ...
    'NumColumns',2);
ylim([-0.05 1.05]);
xlim([1 N]);


for ii = 1:4

    k_star = selectedSteps(ii);
    theta_demo = thetaCon_log(k_star,:);

    [deltaTheta_demo_corr, gamma1_demo, gamma2_demo] = correctDeltaTheta( ...
        theta_demo, deltaTheta_demo, eta, D0, Mth1, Mth2);

    deltaRaw_abs  = abs(deltaTheta_demo);
    deltaCorr_abs = abs(deltaTheta_demo_corr);

    barData = [deltaRaw_abs(:), deltaCorr_abs(:)];

    nexttile;
    b = bar(barData, 'grouped');
    applyLightBarColors(b);

    grid on;
    box on;

    xlabel('Driving unit number');
    ylabel('Angular increment / deg');

    title(sprintf('(%c) Step %d, \\gamma_1 = %.2f, \\gamma_2 = %.2f', ...
        char('b' + ii - 1), k_star, gamma1_demo, gamma2_demo));

    set(gca, ...
        'XTick', 1:6, ...
        'XTickLabel', {'1','2','3','4','5','6'});

    ymax = max(barData(:)) * 1.45;
    if ymax < 1
        ymax = 1;
    end
    ylim([0 ymax]);
    xlim([0.4 6.6]);
    drawSegmentDashedBoxes(ymax, fontName, fontText);

    legend({'|\Delta\theta|','|\Delta\theta^{\prime}|'}, ...
        'Location','southoutside', ...
        'Orientation','horizontal');
end

set(findall(gcf,'-property','FontName'),'FontName',fontName);
set(findall(gcf,'-property','FontSize'),'FontSize',fontAxis);
set(findall(gcf,'Type','axes'),'LineWidth',1.2);

print(gcf, 'Fig11_trend_selected_bars_wide_legend.png', '-dpng', '-r300');

disp('Fig. 11 generated: Fig11_trend_selected_bars_wide_legend.png');


theta_tmp = theta0;
dRmax_pred = zeros(N,1);    
dRmax_corr = zeros(N,1);   

for k = 1:N
    [R1_now, R2_now] = calcRisk(theta_tmp, eta, D0);
    Rmax_now = max(R1_now, R2_now);


    theta_pred_uncorr = theta_tmp + deltaTheta;
    [R1_pred_uncorr, R2_pred_uncorr] = calcRisk(theta_pred_uncorr, eta, D0);
    Rmax_pred_uncorr = max(R1_pred_uncorr, R2_pred_uncorr);


    [deltaThetaCorr_tmp, ~, ~] = correctDeltaTheta( ...
        theta_tmp, deltaTheta, eta, D0, Mth1, Mth2);

    theta_pred_corr = theta_tmp + deltaThetaCorr_tmp;
    [R1_pred_corr, R2_pred_corr] = calcRisk(theta_pred_corr, eta, D0);
    Rmax_pred_corr = max(R1_pred_corr, R2_pred_corr);

    dRmax_pred(k) = Rmax_pred_uncorr - Rmax_now;
    dRmax_corr(k) = Rmax_pred_corr - Rmax_now;


    theta_tmp = theta_pred_corr;
end

figure('Color','w','Position',[120 80 1250 850]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, R1_con, 'LineWidth', lineWidth); hold on;
plot(t, R2_con, 'LineWidth', lineWidth);
plot([t(1), t(end)], [Mth1, Mth1], '--', 'LineWidth', 1.6);
plot([t(1), t(end)], [Mth2, Mth2], '--', 'LineWidth', 1.6);

grid on;
box on;

xlabel('Time / s');
ylabel('Risk indicator');
title('(a) Segment risks after correction');

legend({'R_1','R_2','R_{th1}','R_{th2}'}, ...
    'Location','northwest');

nexttile;
plot(R1_con, gamma1_log, 'LineWidth', lineWidth); hold on;
plot([Mth1, Mth1], [-0.05, 1.05], '--', 'LineWidth', 1.4);
plot([Mth2, Mth2], [-0.05, 1.05], '--', 'LineWidth', 1.4);

grid on;
box on;

xlabel('R_1');
ylabel('\gamma_1');
title('(b) Relation between R_1 and \gamma_1');

legend({'R_1-\gamma_1 relation','R_{th1}','R_{th2}'}, ...
    'Location','southwest');

ylim([-0.05 1.05]);

nexttile;
plot(t, dRmax_pred, 'LineWidth', lineWidth); hold on;
plot(t, dRmax_corr, 'LineWidth', lineWidth);
plot([t(1), t(end)], [0, 0], '--', 'LineWidth', 1.2);

grid on;
box on;

xlabel('Time / s');
ylabel('\Delta R_{max}');
title('(c) Suppression of predicted risk increase');

legend({'Before correction','After correction','Zero increase'}, ...
    'Location','best');

nexttile;
barData = [min(gamma1_log), min(gamma2_log); ...
           mean(gamma1_log), mean(gamma2_log)];

b = bar(barData);
applyLightBarColors(b);

grid on;
box on;

set(gca,'XTickLabel',{'Minimum','Mean'});
ylabel('\gamma');
title('(d) Summary of correction coefficients');

legend({'\gamma_1','\gamma_2'}, ...
    'Location','northwest');

ylim([0 1.05]);

set(findall(gcf,'-property','FontName'),'FontName',fontName);
set(findall(gcf,'-property','FontSize'),'FontSize',fontAxis);
set(findall(gcf,'Type','axes'),'LineWidth',1.2);

print(gcf, 'Fig12_risk_evolution_and_correction_behavior.png', '-dpng', '-r300');

disp('Fig. 12 generated: Fig12_risk_evolution_and_correction_behavior.png');



deltaTheta_case1 = [2, 0, 0, 0, 0, 0];

deltaTheta_case2 = [1.8, -1.4, 0.4, -1.2, 0.9, 0.3];


[tipRaw_case1, tipCon_case1, t_case1] = simulateMotionCase( ...
    theta0, deltaTheta_case1, N, dt, eta, D0, Mth1, Mth2);

[tipRaw_case2, tipCon_case2, t_case2] = simulateMotionCase( ...
    theta0, deltaTheta_case2, N, dt, eta, D0, Mth1, Mth2);

figure('Color','w','Position',[80 60 1450 900]);
tiledlayout(2,3,'Padding','compact','TileSpacing','compact');


nexttile;
plot(tipRaw_case1(:,1), tipRaw_case1(:,3), 'LineWidth', lineWidth); hold on;
plot(tipCon_case1(:,1), tipCon_case1(:,3), 'LineWidth', lineWidth);

plot(tipRaw_case1(1,1), tipRaw_case1(1,3), 'o', ...
    'MarkerSize', 8, 'LineWidth', 1.6);
plot(tipRaw_case1(end,1), tipRaw_case1(end,3), 's', ...
    'MarkerSize', 8, 'LineWidth', 1.6);
plot(tipCon_case1(end,1), tipCon_case1(end,3), '^', ...
    'MarkerSize', 8, 'LineWidth', 1.6);

grid on; box on; axis equal;
xlabel('X / mm');
ylabel('Z / mm');
title('(a) Case 1: X-Z trajectory');
legend({'Unconstrained','Risk-constrained', ...
        'Initial','Unconstrained end','Constrained end'}, ...
        'Location','southwest');


nexttile;
plot(t_case1, tipRaw_case1(:,1), 'LineWidth', lineWidth); hold on;
plot(t_case1, tipCon_case1(:,1), 'LineWidth', lineWidth);

grid on; box on;
xlabel('Time / s');
ylabel('X / mm');
title('(b) Case 1: X-coordinate response');
legend({'Unconstrained','Risk-constrained'}, 'Location','best');

nexttile;
plot(t_case1, tipRaw_case1(:,3), 'LineWidth', lineWidth); hold on;
plot(t_case1, tipCon_case1(:,3), 'LineWidth', lineWidth);

grid on; box on;
xlabel('Time / s');
ylabel('Z / mm');
title('(c) Case 1: Z-coordinate response');
legend({'Unconstrained','Risk-constrained'}, 'Location','best');


nexttile;
plot(tipRaw_case2(:,1), tipRaw_case2(:,3), 'LineWidth', lineWidth); hold on;
plot(tipCon_case2(:,1), tipCon_case2(:,3), 'LineWidth', lineWidth);

plot(tipRaw_case2(1,1), tipRaw_case2(1,3), 'o', ...
    'MarkerSize', 8, 'LineWidth', 1.6);
plot(tipRaw_case2(end,1), tipRaw_case2(end,3), 's', ...
    'MarkerSize', 8, 'LineWidth', 1.6);
plot(tipCon_case2(end,1), tipCon_case2(end,3), '^', ...
    'MarkerSize', 8, 'LineWidth', 1.6);

grid on; box on; axis equal;
xlabel('X / mm');
ylabel('Z / mm');
title('(d) Case 2: X-Z trajectory');
legend({'Unconstrained','Risk-constrained', ...
        'Initial','Unconstrained end','Constrained end'}, ...
        'Location','southwest');

nexttile;
plot(t_case2, tipRaw_case2(:,1), 'LineWidth', lineWidth); hold on;
plot(t_case2, tipCon_case2(:,1), 'LineWidth', lineWidth);

grid on; box on;
xlabel('Time / s');
ylabel('X / mm');
title('(e) Case 2: X-coordinate response');
legend({'Unconstrained','Risk-constrained'}, 'Location','best');

nexttile;
plot(t_case2, tipRaw_case2(:,3), 'LineWidth', lineWidth); hold on;
plot(t_case2, tipCon_case2(:,3), 'LineWidth', lineWidth);

grid on; box on;
xlabel('Time / s');
ylabel('Z / mm');
title('(f) Case 2: Z-coordinate response');
legend({'Unconstrained','Risk-constrained'}, 'Location','best');

set(findall(gcf,'-property','FontName'),'FontName',fontName);
set(findall(gcf,'-property','FontSize'),'FontSize',fontAxis);
set(findall(gcf,'Type','axes'),'LineWidth',1.2);

print(gcf, 'Fig13_two_case_end_tip_motion.png', '-dpng', '-r300');
disp('Fig. 13 generated: Fig13_two_case_end_tip_motion.png');


riskReduction = (max(Rmax_raw) - max(Rmax_con)) / max(Rmax_raw) * 100;

fprintf('Maximum Rmax under unconstrained control = %.4f\n', max(Rmax_raw));
fprintf('Maximum Rmax under risk-constrained control = %.4f\n', max(Rmax_con));
fprintf('Final Rmax under unconstrained control = %.4f\n', Rmax_raw(end));
fprintf('Final Rmax under risk-constrained control = %.4f\n', Rmax_con(end));
fprintf('Risk reduction based on maximum Rmax = %.2f%%\n', riskReduction);
fprintf('Minimum gamma1 = %.4f\n', min(gamma1_log));
fprintf('Minimum gamma2 = %.4f\n', min(gamma2_log));


function applyLightBarColors(b)


    lightColors = [0.72 0.84 0.96;   % light blue
                   0.98 0.80 0.70;   % light orange
                   0.78 0.90 0.78;   % light green
                   0.86 0.80 0.94];  % light purple

    for jj = 1:numel(b)
        colorID = mod(jj-1, size(lightColors,1)) + 1;
        b(jj).FaceColor = lightColors(colorID,:);
        b(jj).EdgeColor = 'none';
    end
end

function drawSegmentDashedBoxes(ymax, fontName, fontText)


    boxHeight = ymax * 0.84;
    yLabelPos = ymax * 0.90;

    rectangle('Position',[0.55, 0, 2.90, boxHeight], ...
        'EdgeColor',[0.35 0.35 0.35], ...
        'LineStyle','--', ...
        'LineWidth',1.2);

    rectangle('Position',[3.55, 0, 2.90, boxHeight], ...
        'EdgeColor',[0.35 0.35 0.35], ...
        'LineStyle','--', ...
        'LineWidth',1.2);

    text(2, yLabelPos, 'segment 1', ...
        'HorizontalAlignment','center', ...
        'FontName',fontName, ...
        'FontSize',fontText);

    text(5, yLabelPos, 'segment 2', ...
        'HorizontalAlignment','center', ...
        'FontName',fontName, ...
        'FontSize',fontText);
end

function [tipRaw_log, tipCon_log, t] = simulateMotionCase( ...
    theta0, deltaTheta, N, dt, eta, D0, Mth1, Mth2)

    theta_raw = theta0;
    theta_con = theta0;

    tipRaw_log = zeros(N+1,3);
    tipCon_log = zeros(N+1,3);

    tipRaw_log(1,:) = calcTipPosition(theta_raw, eta);
    tipCon_log(1,:) = calcTipPosition(theta_con, eta);

    for k = 1:N

        theta_raw = theta_raw + deltaTheta;
        tipRaw_log(k+1,:) = calcTipPosition(theta_raw, eta);

        [deltaThetaCorr, ~, ~] = correctDeltaTheta( ...
            theta_con, deltaTheta, eta, D0, Mth1, Mth2);

        theta_con = theta_con + deltaThetaCorr;
        tipCon_log(k+1,:) = calcTipPosition(theta_con, eta);
    end

    t = (0:N) * dt;
end

function [R1, R2] = calcRisk(theta, eta, D0)
    % Calculate the stiffness-risk indicators of the two segments.

    L = eta * theta;

    L1 = L(1:3);
    L2 = L(4:6);

    D1 = sqrt(mean((L1 - mean(L1)).^2));
    D2 = sqrt(mean((L2 - mean(L2)).^2));

    R1 = D1 / D0;
    R2 = D2 / D0;
end

function [deltaThetaCorr, gamma1, gamma2] = correctDeltaTheta( ...
    theta, deltaTheta, eta, D0, Mth1, Mth2)

    [R1_now, R2_now] = calcRisk(theta, eta, D0);

    theta_pred = theta + deltaTheta;
    [R1_pred, R2_pred] = calcRisk(theta_pred, eta, D0);

    gamma1 = calcGamma(R1_now, R1_pred, Mth1, Mth2);
    gamma2 = calcGamma(R2_now, R2_pred, Mth1, Mth2);

    deltaThetaCorr = deltaTheta;
    deltaThetaCorr(1:3) = gamma1 * deltaTheta(1:3);
    deltaThetaCorr(4:6) = gamma2 * deltaTheta(4:6);
end

function gamma = calcGamma(M_now, M_pred, Mth1, Mth2)


    if M_pred <= M_now

        gamma = 1;

    elseif M_now < Mth1

        gamma = 1;

    elseif M_now >= Mth1 && M_now < Mth2

        gamma = (Mth2 - M_now) / (Mth2 - Mth1);
        gamma = max(min(gamma, 1), 0);

    else

        gamma = 0;
    end
end

function tip = calcTipPosition(theta, eta)


    segLen = 40;         
    nStep = 40;           
    curvatureGain = 2e-4; 

    L = eta * theta;

    Lseg1 = L(1:3);
    Lseg2 = L(4:6);

    [kx1, ky1] = tendonToCurvature(Lseg1, curvatureGain);
    [kx2, ky2] = tendonToCurvature(Lseg2, curvatureGain);

    p = [0; 0; 0];
    R = eye(3);

    [p, R] = integrateSegment(p, R, kx1, ky1, segLen, nStep);
    [p, R] = integrateSegment(p, R, kx2, ky2, segLen, nStep);

    tip = p.';
end

function [kx, ky] = tendonToCurvature(Lseg, curvatureGain)

    phi = [0, 2*pi/3, 4*pi/3];

    Ldev = Lseg - mean(Lseg);

    kx = curvatureGain * sum(Ldev .* cos(phi));
    ky = curvatureGain * sum(Ldev .* sin(phi));
end

function [p, R] = integrateSegment(p, R, kx, ky, segLen, nStep)


    ds = segLen / nStep;

    for ii = 1:nStep

        w = [ky; -kx; 0] * ds;

        dR = rotvecToRotm(w);

        R = R * dR;
        p = p + R * [0; 0; ds];
    end
end

function R = rotvecToRotm(w)

    angle = norm(w);

    if angle < 1e-12
        R = eye(3);
        return;
    end

    u = w / angle;

    ux = [0, -u(3), u(2);
          u(3), 0, -u(1);
          -u(2), u(1), 0];

    R = eye(3) + sin(angle) * ux + (1 - cos(angle)) * (ux * ux);
end
