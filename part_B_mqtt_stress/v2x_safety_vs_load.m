function T = v2x_safety_vs_load(tag, doPlots)
% V2X_SAFETY_VS_LOAD  Turn measured MQTT latency/loss into physical safety margins.
%   T = v2x_safety_vs_load('base_q0_r1')          analyse one run, print results, save figures
%   T = v2x_safety_vs_load('base_q0_r1', false)   silent (used by compare_configs.m)
% Inputs : <tag>_latency.csv and <tag>_summary.csv written by mqtt_stress.py
% Outputs: <tag>_safety_table.csv (+ 4 PNG figures when doPlots = true)

if nargin < 1, tag = 'q0'; end
if nargin < 2, doPlots = true; end
rng(1);

K.v = 25;  K.aL = 8;  K.aF = 6;          % speed (m/s), lead / follower braking (m/s^2)
K.tSys = 0.2;                            % s, automatic braking system reaction
K.probeRate = 10;                        % Hz, must equal --probe-rate used in the stress run
K.gaps = [20 25 30 40];                  % m, initial bumper-to-bumper gaps to test
K.nTrials = 5000;  K.pCrit = 0.01;       % Monte Carlo trials; "unsafe" = collision prob >= 1 %

lat = readtable([tag '_latency.csv']);
sm  = readtable([tag '_summary.csv']);
loads = sm.cars;  nL = numel(loads);
dL = K.v^2/(2*K.aL);                                         % lead stopping distance (m)
budget = (K.gaps + dL - K.v^2/(2*K.aF))/K.v - K.tSys;        % max tolerable warning delay (s)

pCol = zeros(nL, numel(K.gaps));  lossPct = zeros(nL,1);
oneHop = zeros(nL,3);  dly = zeros(nL,3);  dF = zeros(nL,3);

for k = 1:nL
    m = lat.cars == loads(k);
    nP = sm.probes_sent(k);
    lv = NaN(nP,1);  lv(lat.seq(m)) = lat.latency_ms(m);     % latency by probe number (NaN = lost)
    lossPct(k) = 100*(1 - sum(~isnan(lv))/nP);
    got = lv(~isnan(lv));
    oneHop(k,:) = [pct(got,50) pct(got,95) pct(got,99)];

    % Brake event at a random beacon i; follower is warned by the first probe received at/after i.
    delay = zeros(K.nTrials,1);  i0 = randi(max(1,nP-5), K.nTrials, 1);
    for n = 1:K.nTrials
        j = find(~isnan(lv(i0(n):end)), 1);
        if isempty(j), delay(n) = Inf; else, delay(n) = (j-1)/K.probeRate + lv(i0(n)+j-1)/1000; end
    end
    dly(k,:) = [pct(delay,50) pct(delay,95) pct(delay,99)];
    stopF = K.v*(K.tSys + delay) + K.v^2/(2*K.aF);           % follower stopping distance (m)
    dF(k,:) = [pct(stopF,50) pct(stopF,95) pct(stopF,99)];
    for g = 1:numel(K.gaps), pCol(k,g) = mean(stopF > K.gaps(g) + dL); end
end

T = table(loads, sm.offered_msgs_s, sm.achieved_msgs_s, lossPct, oneHop(:,1), oneHop(:,3), ...
    1e3*dly(:,1), 1e3*dly(:,3), dF(:,1), dF(:,3), 'VariableNames', {'Cars','Offered_msgs_s', ...
    'Achieved_msgs_s','ProbeLoss_pct','Lat_p50_ms','Lat_p99_ms','WarnDelay_p50_ms','WarnDelay_p99_ms', ...
    'StopDist_p50_m','StopDist_p99_m'});
colNames = arrayfun(@(g) sprintf('Col_pct_gap%d',g), K.gaps, 'UniformOutput', false);
T = [T array2table(100*pCol, 'VariableNames', colNames)];
writetable(T, [tag '_safety_table.csv']);

if ~doPlots, return; end

disp(T);
fprintf('\nLatency budget (max tolerable warning delay): ');
for g = 1:numel(K.gaps), fprintf('gap %d m -> %.0f ms | ', K.gaps(g), 1e3*budget(g)); end
fprintf('\n');
for g = 1:numel(K.gaps)
    i = find(pCol(:,g) >= K.pCrit, 1);
    if isempty(i), fprintf('Gap %2d m: no unsafe load in tested range (broker not the limiting factor here)\n', K.gaps(g));
    else,          fprintf('Gap %2d m: collision prob >= %.0f %% first at %d cars (%d msg/s offered)\n', ...
                           K.gaps(g), 100*K.pCrit, loads(i), sm.offered_msgs_s(i)); end
end
if any(sm.achieved_msgs_s < 0.95*sm.offered_msgs_s)
    warning('Load generator could not reach the offered rate at some levels: those points are generator-limited.');
end

x = 1:nL;  xl = arrayfun(@num2str, loads, 'UniformOutput', false);
finiteMax = max([dF(isfinite(dF)); 1]);

figure; plot(x, oneHop, '-o', 'LineWidth', 1.6); grid on; set(gca,'XTick',x,'XTickLabel',xl);
xlabel('Simulated cars sending to RSU broker'); ylabel('Probe latency (ms)');
legend('p50','p95','p99','Location','northwest'); title('MQTT broker latency vs load');
saveas(gcf, [tag '_fig1_latency.png']);

figure; yyaxis left; plot(x, lossPct, '-s', 'LineWidth', 1.6); ylabel('Probe loss (%)');
yyaxis right; plot(x, sm.achieved_msgs_s, '-d', 'LineWidth', 1.6); ylabel('Achieved load (msg/s)');
grid on; set(gca,'XTick',x,'XTickLabel',xl); xlabel('Simulated cars'); title('Loss and achieved load');
saveas(gcf, [tag '_fig2_loss_load.png']);

figure; subplot(2,1,1); plot(x, dF(:,1), '-o', x, dF(:,3), '-s', 'LineWidth', 1.6); hold on;
for g = 1:numel(K.gaps), plot([1 nL], (K.gaps(g)+dL)*[1 1], ':', 'LineWidth', 1.2); end
grid on; set(gca,'XTick',x,'XTickLabel',xl); ylabel('Follower stopping distance (m)');
ylim([0 min(150, 1.2*finiteMax)]);
legend([{'p50','p99'}, arrayfun(@(g) sprintf('limit, gap %d m',g), K.gaps, 'UniformOutput', false)], 'Location','northwest');
title('Stopping distance vs load (dotted = gap + lead stopping distance)');
subplot(2,1,2); plot(x, dF(:,1)-dF(1,1), '-o', x, dF(:,3)-dF(1,1), '-s', 'LineWidth', 1.6); grid on;
set(gca,'XTick',x,'XTickLabel',xl); xlabel('Simulated cars'); ylabel('Extra stopping distance (m)');
legend('p50','p99','Location','northwest'); title('Increase vs zero-load baseline');
saveas(gcf, [tag '_fig3_stopping_distance.png']);

figure; plot(x, 100*pCol, '-o', 'LineWidth', 1.6); grid on; set(gca,'XTick',x,'XTickLabel',xl);
xlabel('Simulated cars'); ylabel('Collision probability (%)');
legend(arrayfun(@(g) sprintf('gap %d m',g), K.gaps, 'UniformOutput', false), 'Location','northwest');
title('Collision probability vs broker load'); saveas(gcf, [tag '_fig4_collision.png']);
end

function q = pct(x, p)
    x = sort(x(:));  n = numel(x);
    if n == 0, q = NaN; return; end
    q = x(min(n, max(1, ceil(p/100*n))));
end
