%% compare_configs.m : overlay all test conditions (mean +/- std over repeats) and find the unsafe load
% Needs v2x_safety_vs_load.m and the CSVs from run_matrix.py in the current folder.
clear; clc; close all;

labels  = {'base', 'cpu05', 'loss1'};          % conditions you ran, e.g. {'base','cpu05','loss1'}
qosList = [0 1];
gaps = [20 25 30 40];  gapShow = 25;  pCritPct = 1;       % must match v2x_safety_vs_load.m
v = 25;  aL = 8;  limitDist = gapShow + v^2/(2*aL);       % must match v2x_safety_vs_load.m

curves = [];
for a = 1:numel(labels)
    for q = qosList
        files = dir(sprintf('%s_q%d_r*_latency.csv', labels{a}, q));
        if isempty(files), continue; end
        tags = cellfun(@(f) strrep(f,'_latency.csv',''), {files.name}, 'UniformOutput', false);
        T = cell(numel(tags),1);
        for r = 1:numel(tags), T{r} = v2x_safety_vs_load(tags{r}, false); end
        get = @(nm) cell2mat(cellfun(@(t) t.(nm)', T, 'UniformOutput', false));   % reps x levels

        clear c;
        c.name = sprintf('%s QoS%d (n=%d)', labels{a}, q, numel(tags));
        c.cars = T{1}.Cars';  c.offered = T{1}.Offered_msgs_s';
        M = get('Lat_p99_ms');            c.lat99 = mean(M,1);   c.lat99s = std(M,0,1);
        M = get('ProbeLoss_pct');         c.loss  = mean(M,1);   c.losss  = std(M,0,1);
        M = min(get('StopDist_p99_m'),150); c.stop99 = mean(M,1); c.stop99s = std(M,0,1);
        M = get(sprintf('Col_pct_gap%d',gapShow)); c.col = mean(M,1); c.cols = std(M,0,1);
        c.colGap = zeros(numel(gaps), numel(c.cars));
        for g = 1:numel(gaps), c.colGap(g,:) = mean(get(sprintf('Col_pct_gap%d',gaps(g))),1); end
        if isempty(curves), curves = c; else, curves(end+1) = c; end %#ok<SAGROW>
    end
end
if isempty(curves), error('No files like base_q0_r1_latency.csv found in this folder.'); end

x  = 1:numel(curves(1).cars);
xl = arrayfun(@num2str, curves(1).cars, 'UniformOutput', false);
sty = {'-o','-s','-^','-d','-v','-p','-x','-*'};
names = {curves.name};

figure('Position',[50 50 1200 800]);
subplot(2,2,1); hold on;
for i = 1:numel(curves), errorbar(x, curves(i).lat99, curves(i).lat99s, sty{mod(i-1,8)+1}, 'LineWidth', 1.4); end
grid on; set(gca,'XTick',x,'XTickLabel',xl); xlabel('Simulated cars'); ylabel('p99 probe latency (ms)');
title('Broker latency vs load'); legend(names,'Location','northwest');

subplot(2,2,2); hold on;
for i = 1:numel(curves), errorbar(x, curves(i).loss, curves(i).losss, sty{mod(i-1,8)+1}, 'LineWidth', 1.4); end
grid on; set(gca,'XTick',x,'XTickLabel',xl); xlabel('Simulated cars'); ylabel('Probe loss (%)'); title('Beacon loss vs load');

subplot(2,2,3); hold on;
for i = 1:numel(curves), errorbar(x, curves(i).stop99, curves(i).stop99s, sty{mod(i-1,8)+1}, 'LineWidth', 1.4); end
plot([1 numel(x)], limitDist*[1 1], 'k--', 'LineWidth', 1.6);
grid on; set(gca,'XTick',x,'XTickLabel',xl); xlabel('Simulated cars'); ylabel('p99 follower stopping distance (m)');
title('Stopping distance vs load'); legend([names, {sprintf('limit (gap %d m)',gapShow)}],'Location','northwest');

subplot(2,2,4); hold on;
for i = 1:numel(curves), errorbar(x, curves(i).col, curves(i).cols, sty{mod(i-1,8)+1}, 'LineWidth', 1.4); end
grid on; set(gca,'XTick',x,'XTickLabel',xl); xlabel('Simulated cars'); ylabel('Collision probability (%)');
title(sprintf('Collision probability, gap = %d m',gapShow));
saveas(gcf,'compare_configs.png');

rows = {};
for i = 1:numel(curves)
    for g = 1:numel(gaps)
        k = find(curves(i).colGap(g,:) >= pCritPct, 1);
        if isempty(k), cc = NaN; off = NaN; else, cc = curves(i).cars(k); off = curves(i).offered(k); end
        rows(end+1,:) = {curves(i).name, gaps(g), cc, off}; %#ok<SAGROW>
    end
end
S = cell2table(rows, 'VariableNames', {'Condition','Gap_m','FirstUnsafe_Cars','Offered_msgs_s'});
disp(S); writetable(S,'final_summary.csv');
fprintf('NaN in FirstUnsafe_Cars = no unsafe load found in the tested range for that condition and gap.\n');
