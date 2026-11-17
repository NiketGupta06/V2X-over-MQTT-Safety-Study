%% v2x_sim.m : Minimal V2X (V2V + V2I) simulation, base MATLAB only
clear; clc; close all; rng(1);

%% ---------------- Parameters ----------------
P.roadLen  = 1000;           % m, ring road (wraps around => constant density)
P.lanes    = 2;  P.laneW = 3.5;
P.simTime  = 20;             % s
P.dt       = 0.1;            % s, beacon period (10 Hz)
P.txPower  = 23;             % dBm, vehicle
P.rsuPower = 30;             % dBm, RSU
P.PL0      = 47.86;          % dB, free-space loss at 1 m, 5.9 GHz
P.n        = 2.7;            % path-loss exponent
P.shadowStd= 3;              % dB
P.noise    = -95;            % dBm
P.sinrTh   = 5;              % dB, reception threshold
P.nSlots   = 40;             % random-access slots per beacon period
P.slotDur  = 0.5e-3;         % s
P.airtime  = 0.4e-3;         % s, ~300 B at 6 Mbps
P.pktBytes = 300;
P.rsuX     = 500;  P.rsuOffset = 5;   % RSU position (m), roadside offset (m)
% Collision-warning application
P.v        = 25;             % m/s, both vehicles
P.aLead    = 8;              % m/s^2, lead braking
P.aFollow  = 6;              % m/s^2, follower braking
P.tDriver  = 1.5;            % s, mean human reaction (brake-light perception)
P.tSys     = 0.2;            % s, automatic system reaction after V2X warning
P.nTrials  = 2000;
densities  = [10 20 40 60 80 100];   % vehicles per km

%% ---------------- Run network simulation per density ----------------
for k = 1:numel(densities)
    out(k) = runSim(densities(k), P); %#ok<SAGROW>
    fprintf('Density %3d veh/km -> PDR(<=300m) = %.1f %%, latency = %.2f ms\n', ...
        densities(k), 100*out(k).pdr300, out(k).latMean);
end

%% ---------------- Results table ----------------
T = table(densities', 100*[out.pdr300]', [out.latMean]', [out.lat95]', [out.thr]', ...
    'VariableNames', {'Density_veh_per_km','PDR_300m_percent','MeanLatency_ms','Lat95_ms','Throughput_kbps_per_veh'});
disp(T); writetable(T, 'results.csv');

%% ---------------- Fig 1: PDR vs distance ----------------
figure(1); hold on; grid on;
for k = 1:numel(densities)
    plot(out(k).centers, 100*out(k).pdr, '-o', 'LineWidth', 1.5);
end
xlabel('Distance (m)'); ylabel('PDR (%)'); title('V2V PDR vs distance');
legend(arrayfun(@(d) sprintf('%d veh/km', d), densities, 'UniformOutput', false), 'Location', 'southwest');
saveas(gcf, 'fig1_pdr_vs_distance.png');

%% ---------------- Fig 2: PDR and latency vs density ----------------
figure(2);
yyaxis left;  plot(densities, 100*[out.pdr300], '-o', 'LineWidth', 1.5); ylabel('PDR within 300 m (%)');
yyaxis right; plot(densities, [out.latMean], '-s', 'LineWidth', 1.5);    ylabel('Mean latency (ms)');
xlabel('Vehicle density (veh/km)'); title('Effect of traffic density'); grid on;
saveas(gcf, 'fig2_density_effect.png');

%% ---------------- Fig 3: V2I (RSU to vehicle) PDR vs distance ----------------
figure(3);
plot(out(3).centers, 100*out(3).rsuPdr, '-o', 'LineWidth', 1.5); grid on;
xlabel('Distance from RSU (m)'); ylabel('PDR (%)'); title('V2I (RSU to vehicle) PDR vs distance');
saveas(gcf, 'fig3_v2i_pdr.png');

%% ---------------- Collision-warning application ----------------
idx   = 3;                                   % use the 40 veh/km PDR curve
gaps  = 10:5:80;                             % initial gap (m)
pColNo = zeros(size(gaps)); pColV2X = zeros(size(gaps));
extra = P.v^2/(2*P.aFollow) - P.v^2/(2*P.aLead);   % extra stopping distance of follower
for g = 1:numel(gaps)
    pdrG = interp1(out(idx).centers, out(idx).pdr, gaps(g), 'linear', 'extrap');
    pdrG = min(max(pdrG, 0.01), 1);
    colNo = 0; colV2X = 0;
    for t = 1:P.nTrials
        % Without V2X: human reacts to brake lights
        tr0 = max(0.7, P.tDriver + 0.3*randn);
        colNo = colNo + (P.v*tr0 + extra > gaps(g));
        % With V2X: first successfully received beacon triggers automatic reaction
        kTx = 1; while rand > pdrG && kTx < 50, kTx = kTx + 1; end
        tr1 = P.tSys + (kTx-1)*P.dt + out(idx).latMean*1e-3;
        colV2X = colV2X + (P.v*tr1 + extra > gaps(g));
    end
    pColNo(g) = colNo/P.nTrials; pColV2X(g) = colV2X/P.nTrials;
end
figure(4); plot(gaps, 100*pColNo, '-o', gaps, 100*pColV2X, '-s', 'LineWidth', 1.5); grid on;
xlabel('Initial gap to lead vehicle (m)'); ylabel('Collision probability (%)');
legend('Without V2X (human driver)', 'With V2X warning', 'Location', 'northeast');
title('Cooperative collision warning benefit'); saveas(gcf, 'fig4_collision.png');

%% ---------------- Fig 5: road snapshot ----------------
figure(5); hold on; axis equal; grid on;
plot(out(3).x, out(3).y, 'bs', 'MarkerFaceColor', 'b');
plot(P.rsuX, -P.rsuOffset, 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
th = linspace(0, 2*pi, 100); R = 258;        % approx. coverage radius
plot(P.rsuX + R*cos(th), -P.rsuOffset + R*sin(th), 'r--');
xlim([0 P.roadLen]); ylim([-R-10 R+10]); xlabel('x (m)'); ylabel('y (m)');
legend('Vehicles', 'RSU', 'Approx. coverage'); title('Highway snapshot (40 veh/km)');
saveas(gcf, 'fig5_snapshot.png');

%% ======================= Local functions =======================
function out = runSim(N, P)
    L = P.roadLen; thLin = 10^(P.sinrTh/10); noise = 10^(P.noise/10);
    x = rand(N,1)*L; y = (randi(P.lanes,N,1)-1)*P.laneW; v = 20 + 10*rand(N,1);
    edges = 0:50:500; nb = numel(edges)-1;
    tx = zeros(nb,1); rx = zeros(nb,1); rtx = zeros(nb,1); rrx = zeros(nb,1);
    lat = []; steps = round(P.simTime/P.dt);
    for t = 1:steps
        x  = mod(x + v*P.dt, L);
        dx = abs(x - x'); dx = min(dx, L - dx);          % ring distance
        D  = sqrt(dx.^2 + (y - y').^2); D(1:N+1:end) = Inf;
        PL = P.PL0 + 10*P.n*log10(max(D,1));
        Prx = 10.^((P.txPower - PL + P.shadowStd*randn(N,N))/10);   % row = tx, col = rx (mW)
        Prx(1:N+1:end) = 0;
        slot = randi(P.nSlots, N, 1);
        lat  = [lat; slot*P.slotDur + P.airtime]; %#ok<AGROW>
        for j = 1:N                                       % receiver j
            tot    = accumarray(slot, Prx(:,j), [P.nSlots 1]);
            interf = tot(slot) - Prx(:,j);                % same-slot interferers
            sinr   = Prx(:,j) ./ (noise + interf);
            ok     = (sinr >= thLin) & (slot ~= slot(j)); % half-duplex
            bin    = discretize(D(:,j), edges);
            valid  = ~isnan(bin);
            tx = tx + accumarray(bin(valid), 1, [nb 1]);
            rx = rx + accumarray(bin(valid), double(ok(valid)), [nb 1]);
        end
        % V2I: RSU broadcast (own slot, only noise)
        dxr = abs(x - P.rsuX); dxr = min(dxr, L - dxr);
        dr  = sqrt(dxr.^2 + (y + P.rsuOffset).^2);
        PrR = P.rsuPower - (P.PL0 + 10*P.n*log10(max(dr,1))) + P.shadowStd*randn(N,1);
        okR = (PrR - P.noise) >= P.sinrTh;
        bR  = discretize(dr, edges); vR = ~isnan(bR);
        rtx = rtx + accumarray(bR(vR), 1, [nb 1]);
        rrx = rrx + accumarray(bR(vR), double(okR(vR)), [nb 1]);
    end
    ls = sort(lat);
    out.centers = (edges(1:end-1) + edges(2:end))/2;
    out.pdr     = rx ./ max(tx,1);
    out.rsuPdr  = rrx ./ max(rtx,1);
    out.pdr300  = sum(rx(1:6)) / sum(tx(1:6));
    out.latMean = mean(lat)*1e3;
    out.lat95   = ls(ceil(0.95*numel(ls)))*1e3;
    out.thr     = sum(rx)*P.pktBytes*8 / (N*P.simTime) / 1e3;   % kbps received per vehicle
    out.x = x; out.y = y;
end
