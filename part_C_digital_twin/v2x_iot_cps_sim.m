%% v2x_iot_cps_sim.m : IoT cyber-physical V2X digital twin
%  - Real MQTT (Eclipse Mosquitto) via MATLAB mqttclient  (V2V, V2I, I2V, V2N topics)
%  - 3D cinematic view + top-down MQTT network graph + live KPI dashboard
%  - Artificial network stress (delay + packet drop) applied to MQTT payload processing
%  - NO machine learning / AI: pure IoT networking + kinematics
%
%  REQUIREMENTS: MATLAB R2021a+ with Industrial Communication Toolbox (mqttclient)
%                Mosquitto broker running locally:  mosquitto -v
%  If the toolbox or broker is missing, the script prints a warning and falls back to an
%  in-process LOOPBACK broker (same topics/callbacks, but NOT real MQTT traffic).
clear; clc; close all; rng(3);

%% ---------------- Parameters ----------------
% Scenario / kinematics
P.gap = 40;  P.v0 = 25;  P.aLead = 8;  P.aFollow = 6;
P.tBrake = 3;  P.tHuman = 1.5;  P.tSys = 0.2;
P.carLen = 4.5;  P.carW = 1.9;  P.rsuX = 340;
P.dt = 0.05;  P.tEnd = 10;  P.framePause = 0.02;  P.saveVideo = false;
% MQTT
P.useMQTT = true;  P.broker = 'tcp://localhost';  P.port = 1883;
P.beacon = 0.1;        % s, V2V telemetry + V2I event period (10 Hz)
P.trafficRate = 2;     % Hz, background traffic telemetry (network load)
P.rsuProc = 0.02;      % s, RSU edge processing time
P.backhaul = 0.02;     % s, extra RSU -> cloud (V2N) delay
% Network stress applied inside MQTT subscriber callbacks
P.dropBase = 0;  P.delayBase = 0.2156;  P.jitter = 0.006;   % normal conditions
P.stress = [3.2 5.0];  P.dropStress = 0;  P.delayStress = 0.3005;  % congestion window (sim s)
% KPI windows
P.pdrWin = 2;  P.guard = 0.15;

r(1) = runScene(false, P);   % manual driver
r(2) = runScene(true,  P);   % V2X + MQTT

%% ---------------- Summary + comparison ----------------
fprintf('\n=========== SUMMARY (initial gap = %d m) ===========\n', P.gap);
for k = 1:2
    fprintf('%-22s collision=%d | min gap=%5.1f m | min TTC=%4.2f s | V2V lat=%5.1f ms | V2V PDR=%5.1f %% | msgs=%d | real MQTT=%d\n', ...
        r(k).mode, r(k).collided, r(k).minGap, r(k).minTTC, r(k).meanLat, r(k).pdr, r(k).msgs, r(k).real);
end
figure('Color','w','Name','Manual vs V2X','Position',[100 100 900 600]);
subplot(2,1,1); plot(r(1).t, r(1).gap, 'r-', r(2).t, r(2).gap, 'b-', 'LineWidth', 1.8); grid on;
ylabel('Gap (m)'); legend('Manual driver','V2X (MQTT)','Location','best'); title('Inter-vehicle gap');
subplot(2,1,2); plot(r(1).t, r(1).ttc, 'r-', r(2).t, r(2).ttc, 'b-', 'LineWidth', 1.8); grid on;
ylabel('TTC (s, capped at 10)'); xlabel('Time (s)'); title('Time-to-collision');

%% ======================= Scene / simulation =======================
function res = runScene(useV2X, P)
    L = 600;  nSteps = round(P.tEnd/P.dt);
    if useV2X, modeStr = 'V2X CONNECTED MODE'; else, modeStr = 'MANUAL DRIVER MODE'; end
    cBg = [0.02 0.02 0.05];  cTxt = [0.7 0.9 1];  cPan = [0.04 0.06 0.10];

    fig = figure('Color',cBg,'Position',[20 20 1500 860],'Name',modeStr,'NumberTitle','off');

    % ---------- shared state (callbacks write here via fig.UserData) ----------
    S.T0 = tic;  S.msgCount = 0;  S.log = {};
    S.q = zeros(0,3);  S.lastRelay = -inf;             % RSU queue rows: [rxTime origTime relayed]
    S.warnV2V = inf;  S.warnI2V = inf;                  % effective sim time a warning reached the follower
    S.latV2V = NaN;  S.latE2E = NaN;  S.latV2N = NaN;  S.cloudRx = 0;
    N = 2000;
    S.nV2V = 0;  S.v2vTx = zeros(1,N);  S.v2vOk = false(1,N);
    S.nV2I = 0;  S.v2iTx = zeros(1,N);  S.v2iOk = false(1,N);
    S.nI2V = 0;  S.i2vTx = zeros(1,N);  S.i2vOk = false(1,N);
    S.nV2N = 0;
    S.lastOk = struct('V2V',-inf,'V2I',-inf,'I2V',-inf,'V2N',-inf);
    S.lastDrop = S.lastOk;
    fig.UserData = S;

    % ---------- MQTT clients + topics ----------
    net = struct('real',false,'fig',fig,'c',struct());
    if useV2X
        net = netInit(P, fig);
        netSub(net,'foll', 'v2x/v2v/lead/telemetry', @(tp,m) onMsg(fig,P,'v2v', tp,m), 0);  % V2V
        netSub(net,'rsu',  'v2x/v2i/lead/event',     @(tp,m) onMsg(fig,P,'v2i', tp,m), 1);  % V2I uplink
        netSub(net,'rsu',  'v2x/v2v/+/telemetry',    @(tp,m) onMsg(fig,P,'load',tp,m), 0);  % RSU channel-load monitor
        netSub(net,'foll', 'v2x/i2v/rsu/warning',    @(tp,m) onMsg(fig,P,'i2v', tp,m), 1);  % RSU -> vehicle warning
        netSub(net,'cloud','v2x/v2n/cloud/report',   @(tp,m) onMsg(fig,P,'v2n', tp,m), 1);  % V2N report
    end

    % ================= SUBPLOT 1: 3D cinematic view =================
    ax3 = axes('Parent',fig,'Position',[0.01 0.36 0.60 0.63]); hold(ax3,'on');
    axis(ax3,'off'); daspect(ax3,[1 1 1]); set(ax3,'Clipping','off','Projection','perspective');
    xlim(ax3,[-100 800]); ylim(ax3,[-40 40]); zlim(ax3,[-1 20]);
    [Xg,Yg] = meshgrid(-100:20:800,-40:20:40);
    surf(ax3,Xg,Yg,-0.05*ones(size(Xg)),'FaceColor','none','EdgeColor',[0 0.25 0.4],'EdgeAlpha',0.5);
    patch('Parent',ax3,'XData',[-100 800 800 -100],'YData',[-2 -2 5.5 5.5],'ZData',[0 0 0 0], ...
          'FaceColor',[0.08 0.08 0.1],'EdgeColor','none');
    plot3(ax3,[-100 800],[1.75 1.75],[0.02 0.02],'--','Color',[0 1 1],'LineWidth',1.5);
    plot3(ax3,[-100 800],[-2 -2],[0.02 0.02],'-','Color',[0 0.5 1],'LineWidth',2);
    plot3(ax3,[-100 800],[5.5 5.5],[0.02 0.02],'-','Color',[0 0.5 1],'LineWidth',2);
    light('Parent',ax3,'Position',[200 40 100],'Style','local','Color',[0.5 0.8 1]);
    light('Parent',ax3,'Position',[P.rsuX 0 50],'Style','local','Color',[1 0.3 0.3]);

    % RSU tower with visible states (idle / processing V2I / broadcasting I2V)
    boxPatch(ax3, P.rsuX, -5, 4, 0.3, 0.3, 8, [0.1 0.1 0.2]);
    rsuCore = boxPatch(ax3, P.rsuX, -5, 8.4, 1.4, 1.4, 0.9, [0.1 0.1 0.15]);
    hRsuTxt = text(ax3, P.rsuX, -5, 11, 'EDGE RSU : IDLE', 'FontSize',10,'FontName','Consolas', ...
                   'FontWeight','bold','Color',[0 1 1],'HorizontalAlignment','center');
    hRsuRing = plot3(ax3,nan,nan,nan,'-','Color',[1 0 0.5],'LineWidth',3);

    lead = makeNeonCar(ax3,[0 0.6 1],P.carLen,P.carW);
    foll = makeNeonCar(ax3,[0.2 1 0.2],P.carLen,P.carW);
    nT = 7;  xT0 = rand(nT,1)*L;  vT = 22 + 8*rand(nT,1);
    for i = 1:nT, traf(i) = makeNeonCar(ax3,[0.4 0.4 0.5],P.carLen,P.carW); end %#ok<AGROW>

    hRing1 = plot3(ax3,nan,nan,nan,'-','Color',[1 0.2 0.5],'LineWidth',4);
    hRing2 = plot3(ax3,nan,nan,nan,'-','Color',[1 0.2 0.5],'LineWidth',1.5);
    hV2V3  = plot3(ax3,nan,nan,nan,'-','Color',[0 1 0.5],'LineWidth',3);
    hV2I3  = plot3(ax3,nan,nan,nan,'--','Color',[0 1 1],'LineWidth',2);
    th = linspace(0,2*pi,80);
    dirv = [-0.55 0.70 0.45]; dirv = dirv/norm(dirv);    % wide isometric camera on the +y side

    % HUD + titles
    hud = annotation(fig,'textbox',[0.015 0.69 0.22 0.29],'String','','BackgroundColor',[0.05 0.1 0.15], ...
        'Color',[0 1 1],'EdgeColor',[0 0.8 1],'FaceAlpha',0.7,'LineWidth',1.5,'FontName','Consolas', ...
        'FontSize',10,'FontWeight','bold','Interpreter','tex','FitBoxToText','off');
    annotation(fig,'textbox',[0.18 0.93 0.28 0.06],'String',modeStr,'BackgroundColor','none','Color',[1 1 1], ...
        'EdgeColor','none','FontName','Consolas','FontSize',16,'FontWeight','bold', ...
        'HorizontalAlignment','center','FitBoxToText','off');

    % ================= SUBPLOT 2: top-down MQTT network graph =================
    axN = axes('Parent',fig,'Position',[0.63 0.56 0.36 0.42],'Color',cPan,'XColor',cTxt,'YColor',cTxt, ...
               'XTick',[],'YTick',[],'Box','on'); hold(axN,'on'); ylim(axN,[-12 62]);
    hRoad = patch('Parent',axN,'XData',[0 1 1 0],'YData',[-2 -2 5.5 5.5],'FaceColor',[0.12 0.12 0.15],'EdgeColor','none');
    cLink = struct('V2V',[0 1 0.5],'V2I',[0 1 1],'I2V',[1 0.8 0],'V2N',[1 0.2 0.8]);
    hSp = plot(axN,nan,nan,':','Color',[0.25 0.35 0.45]);
    for nm = fieldnames(cLink)', hLnk.(nm{1}) = plot(axN,nan,nan,'-','Color',[0.25 0.35 0.45]); end %#ok<AGROW>
    hTraf  = plot(axN,nan,nan,'s','MarkerSize',6,'MarkerFaceColor',[0.4 0.4 0.5],'MarkerEdgeColor',[0.6 0.6 0.7],'LineStyle','none');
    hLeadN = plot(axN,nan,nan,'s','MarkerSize',12,'MarkerFaceColor',[0 0.6 1],'MarkerEdgeColor','w','LineStyle','none');
    hFollN = plot(axN,nan,nan,'s','MarkerSize',12,'MarkerFaceColor',[0.2 1 0.2],'MarkerEdgeColor','w','LineStyle','none');
    hRsuN  = plot(axN,nan,nan,'^','MarkerSize',16,'MarkerFaceColor',[0.1 0.1 0.2],'MarkerEdgeColor',[0 1 1],'LineStyle','none');
    hBrkN  = plot(axN,nan,nan,'h','MarkerSize',20,'MarkerFaceColor',[1 0.5 0],'MarkerEdgeColor','w','LineStyle','none');
    hCldN  = plot(axN,nan,nan,'o','MarkerSize',20,'MarkerFaceColor',[0.6 0.3 1],'MarkerEdgeColor','w','LineStyle','none');
    tx = @(s,c) text(axN,0,0,s,'Color',c,'FontName','Consolas','FontSize',8,'FontWeight','bold');
    tLead = tx('LEAD',[0.6 0.9 1]);  tFoll = tx('FOLLOWER',[0.6 1 0.6]);  tRsu = tx('RSU',[0 1 1]);
    tBrk = tx('MOSQUITTO BROKER',[1 0.7 0.3]);  tCld = tx('CLOUD',[0.8 0.6 1]);
    lg = {'V2V',cLink.V2V,0.93; 'V2I',cLink.V2I,0.88; 'I2V',cLink.I2V,0.83; 'V2N',cLink.V2N,0.78};
    for i = 1:4
        text(axN,0.02,lg{i,3},['\bullet ' lg{i,1}],'Units','normalized','Color',lg{i,2},'FontName','Consolas','FontWeight','bold','FontSize',9);
    end
    hStress = text(axN,0.98,0.93,'','Units','normalized','Color',[1 0.2 0.2],'FontName','Consolas', ...
                   'FontWeight','bold','FontSize',9,'HorizontalAlignment','right');

    % MQTT live log
    hLog = annotation(fig,'textbox',[0.63 0.36 0.36 0.19],'String','','BackgroundColor',cPan,'Color',[0.6 1 0.8], ...
        'EdgeColor',[0 0.5 0.7],'FontName','Consolas','FontSize',8,'Interpreter','none','FitBoxToText','off');

    % ================= SUBPLOT 3: live KPI dashboard =================
    axA = axes('Parent',fig,'Position',[0.05 0.07 0.43 0.24],'Color',cPan,'XColor',cTxt,'Box','on'); hold(axA,'on');
    yyaxis(axA,'left');  set(axA,'YColor',cTxt);
    hLatV = plot(axA,nan,nan,'-','Color',[0 1 0.5],'LineWidth',1.8);
    hLatI = plot(axA,nan,nan,'-','Color',[0 1 1],'LineWidth',1.8);
    ylabel(axA,'Latency (ms)'); ylim(axA,[0 300]);
    yyaxis(axA,'right'); set(axA,'YColor',cTxt);
    hPdrV = plot(axA,nan,nan,'--','Color',[1 0.8 0],'LineWidth',1.8);
    hPdrI = plot(axA,nan,nan,'--','Color',[1 0.3 0.8],'LineWidth',1.8);
    ylabel(axA,'PDR (%)'); ylim(axA,[0 105]); xlim(axA,[0 P.tEnd]); grid(axA,'on');
    xlabel(axA,'Time (s)'); title(axA,'PLOT 1: NETWORK METRICS (E2E latency, PDR)','Color','w','FontSize',10);
    legend(axA,[hLatV hLatI hPdrV hPdrI],{'V2V latency','V2I->I2V E2E latency','V2V PDR','V2I PDR'}, ...
           'TextColor','w','Color',cPan,'FontSize',7,'Location','southeast');

    axB = axes('Parent',fig,'Position',[0.55 0.07 0.43 0.24],'Color',cPan,'XColor',cTxt,'Box','on'); hold(axB,'on');
    yyaxis(axB,'left'); set(axB,'YColor',cTxt);
    hGap = plot(axB,nan,nan,'-','Color',[0 0.8 1],'LineWidth',1.8);
    ylabel(axB,'Gap (m)'); ylim(axB,[0 P.gap+20]);
    yyaxis(axB,'right'); set(axB,'YColor',cTxt);
    hTtc = plot(axB,nan,nan,'-','Color',[1 0.6 0],'LineWidth',1.8);
    plot(axB,[0 P.tEnd],[1.5 1.5],':','Color',[1 0.3 0.3]);
    ylabel(axB,'TTC (s, cap 10)'); ylim(axB,[0 10.5]); xlim(axB,[0 P.tEnd]); grid(axB,'on');
    xlabel(axB,'Time (s)'); title(axB,'PLOT 2: KINEMATIC METRICS (gap, TTC; red = 1.5 s critical)','Color','w','FontSize',10);
    legend(axB,[hGap hTtc],{'Inter-vehicle gap','TTC'},'TextColor','w','Color',cPan,'FontSize',7,'Location','northeast');

    % ---------- history arrays ----------
    dT = NaN(1,nSteps); dLatV = dT; dLatI = dT; dPdrV = dT; dPdrI = dT; dGap = dT; dTtc = dT;

    % ---------- initial state ----------
    xL = 200; xF = xL - P.gap - P.carLen; vL = P.v0; vF = P.v0;
    collided = false; minGap = inf; tCol = inf; camT = []; camD = [];
    if useV2X
        if net.real, netTag = 'MQTT: LIVE MOSQUITTO BROKER'; else, netTag = 'MQTT: LOOPBACK (no broker)'; end
    else
        netTag = 'NETWORK: OFFLINE';
    end
    if P.saveVideo
        vw = VideoWriter(sprintf('v2x_cps_%s.mp4', strrep(modeStr,' ','_')),'MPEG-4'); open(vw);
    end

    % ================= main loop =================
    for s = 1:nSteps
        t = s*P.dt;
        S = fig.UserData;

        % ----- follower decision: human reaction OR first V2X warning, whichever is earlier -----
        tWarn = min(S.warnV2V, S.warnI2V);
        tReact = P.tBrake + P.tHuman;
        if useV2X, tReact = min(tReact, tWarn + P.tSys); end

        % ----- kinematics -----
        if t >= P.tBrake, vL = max(0, vL - P.aLead*P.dt);   end
        if t >= tReact,   vF = max(0, vF - P.aFollow*P.dt); end
        xL = xL + vL*P.dt;  xF = xF + vF*P.dt;
        gapNow = xL - xF - P.carLen;
        if gapNow <= 0 && ~collided, collided = true; tCol = t; end
        if collided, xF = xL - P.carLen; vF = vL; gapNow = 0; end
        minGap = min(minGap, gapNow);
        braking = (t >= P.tBrake) && (vL > 0);

        % ----- MQTT publishing (V2X mode only) -----
        if useV2X
            if floor(t/P.beacon + 1e-9) > floor((t-P.dt)/P.beacon + 1e-9)       % 10 Hz tick
                S.nV2V = S.nV2V + 1;  S.v2vTx(S.nV2V) = t;
                pl = jsonencode(struct('id','lead','seq',S.nV2V,'tSim',t,'tWall',toc(S.T0), ...
                        'x',xL,'v',vL,'a',-P.aLead*braking,'brake',double(braking)));
                S = pubMsg(fig,net,S,'lead','v2x/v2v/lead/telemetry',pl,0);
                pl = jsonencode(struct('id','foll','seq',s,'tSim',t,'tWall',toc(S.T0), ...
                        'x',xF,'v',vF,'a',0,'brake',double(t >= tReact && vF > 0)));
                S = pubMsg(fig,net,S,'foll','v2x/v2v/foll/telemetry',pl,0);
                if braking                                                       % V2I hard-brake event (uplink to RSU)
                    S.nV2I = S.nV2I + 1;  S.v2iTx(S.nV2I) = t;
                    pl = jsonencode(struct('type','HARD_BRAKE','seq',S.nV2I,'tSim',t,'tWall',toc(S.T0),'x',xL,'v',vL));
                    S = pubMsg(fig,net,S,'lead','v2x/v2i/lead/event',pl,1);
                end
            end
            if floor(t*P.trafficRate + 1e-9) > floor((t-P.dt)*P.trafficRate + 1e-9)   % background load
                for i = 1:nT
                    pl = jsonencode(struct('id',sprintf('T%d',i),'seq',s,'tSim',t,'tWall',toc(S.T0), ...
                            'x',mod(xT0(i)+vT(i)*t,L),'v',vT(i),'a',0,'brake',0));
                    S = pubMsg(fig,net,S,'traf',sprintf('v2x/v2v/T%d/telemetry',i),pl,0);
                end
            end
            due = find(S.q(:,3) == 0 & S.q(:,1) + P.rsuProc <= t)';              % RSU relays after edge processing
            for k = due
                tRelay = S.q(k,1) + P.rsuProc;  S.q(k,3) = 1;
                S.nI2V = S.nI2V + 1;  S.i2vTx(S.nI2V) = tRelay;  S.lastRelay = tRelay;
                pl = jsonencode(struct('type','I2V_WARNING','seq',S.nI2V,'tSim',tRelay,'tOrig',S.q(k,2), ...
                        'tWall',toc(S.T0),'x',xL));
                S = pubMsg(fig,net,S,'rsu','v2x/i2v/rsu/warning',pl,1);
            end
            if floor(t + 1e-9) > floor(t - P.dt + 1e-9)                          % 1 Hz RSU -> cloud
                S.nV2N = S.nV2N + 1;
                pl = jsonencode(struct('type','RSU_REPORT','seq',S.nV2N,'tSim',t,'tWall',toc(S.T0), ...
                        'load',S.msgCount,'events',S.nV2I));
                S = pubMsg(fig,net,S,'rsu','v2x/v2n/cloud/report',pl,1);
            end
            fig.UserData = S;  pause(0.01);  S = fig.UserData;                   % flush callbacks BEFORE rendering
        end

        % ----- KPIs -----
        pdrV = winPDR(S.v2vTx,S.v2vOk,S.nV2V,t,P.pdrWin,P.guard);
        pdrI = winPDR(S.v2iTx,S.v2iOk,S.nV2I,t,P.pdrWin,P.guard);
        vc = vF - vL;
        if gapNow <= 0, ttc = 0; elseif vc > 0.05, ttc = min(gapNow/vc,10); else, ttc = 10; end
        dT(s) = t; dLatV(s) = S.latV2V; dLatI(s) = S.latE2E; dPdrV(s) = pdrV; dPdrI(s) = pdrI;
        dGap(s) = gapNow; dTtc(s) = ttc;

        % ----- RSU state: 0 idle, 2 processing V2I, 3 broadcasting I2V -----
        rsuState = 0;
        if useV2X
            if any(S.q(:,1) <= t & t < S.q(:,1) + P.rsuProc + 0.2), rsuState = 2;
            elseif t - S.lastRelay < 0.3,                           rsuState = 3; end
        end
        switch rsuState
            case 0, cR = [0.1 0.1 0.15]; sR = 'IDLE';
            case 2, cR = [1 0 0.5];      sR = 'PROCESSING V2I';
            case 3, cR = [0 1 1];        sR = 'BROADCASTING I2V';
        end
        if ~useV2X, sR = 'OFFLINE'; end
        set(rsuCore,'FaceVertexCData',shadeCol(cR+[0.02 0.02 0.02]));
        set(hRsuTxt,'String',['EDGE RSU : ' sR],'Color',max(cR,[0.3 0.6 0.6]));
        if rsuState > 0
            rr = 2.5 + 2*mod(t*4,1);
            set(hRsuRing,'XData',P.rsuX+rr*cos(th),'YData',-5+rr*sin(th),'ZData',8.4*ones(size(th)),'Color',cR);
        else
            set(hRsuRing,'XData',nan,'YData',nan,'ZData',nan);
        end

        % ----- 3D vehicles -----
        set(lead.tf,'Matrix',makehgtform('translate',[xL 0 0]));
        set(foll.tf,'Matrix',makehgtform('translate',[xF 0 0]));
        for i = 1:nT, set(traf(i).tf,'Matrix',makehgtform('translate',[mod(xT0(i)+vT(i)*t,L) 3.5 0])); end
        if t >= P.tBrake || collided, triggerBrakes(lead); end
        if collided || t >= tReact, triggerBrakes(foll);
        elseif useV2X && t >= tWarn, set(foll.body,'FaceVertexCData',shadeCol([1 0.8 0])); end

        set([hRing1 hRing2 hV2V3 hV2I3],'XData',nan,'YData',nan,'ZData',nan);
        if useV2X && t >= P.tBrake && t < tWarn
            rr = mod((t-P.tBrake)*80,50) + 2;
            set(hRing1,'XData',xL+rr*cos(th),'YData',rr*sin(th),'ZData',1.5*ones(size(th)));
            set(hRing2,'XData',xL+0.7*rr*cos(th),'YData',0.7*rr*sin(th),'ZData',1.5*ones(size(th)));
            set(hV2I3,'XData',[xL P.rsuX],'YData',[0 -5],'ZData',[2 8.4]);
        elseif useV2X && t >= tWarn && t < tWarn + 1.5 && ~collided
            set(hV2V3,'XData',[xL xF],'YData',[0 0],'ZData',[2.4 2.4]);
            set(hV2I3,'XData',[xL P.rsuX],'YData',[0 -5],'ZData',[2 8.4]);
        end

        % ----- camera: wide isometric view that always keeps both cars AND the RSU in frame -----
        tgt = [(xF + max(xL,P.rsuX))/2, 1, 3];
        dist = 1.3*(max(xL,P.rsuX) - xF + 50);
        if isempty(camT), camT = tgt; camD = dist; else, camT = camT + 0.2*(tgt-camT); camD = camD + 0.2*(dist-camD); end
        set(ax3,'CameraTarget',camT,'CameraPosition',camT + camD*dirv,'CameraUpVector',[0 0 1],'CameraViewAngle',25);

        % ----- network graph -----
        xr = [min([xF P.rsuX])-30, max([xL P.rsuX])+30];  cx = mean(xr);
        brk = [cx 30];  cld = [xr(2)-18 50];
        set(axN,'XLim',xr);
        set(hRoad,'XData',[xr(1) xr(2) xr(2) xr(1)]);
        set(hLeadN,'XData',xL,'YData',0);  set(hFollN,'XData',xF,'YData',0);
        set(hTraf,'XData',mod(xT0+vT*t,L),'YData',3.5*ones(nT,1));
        set(hRsuN,'XData',P.rsuX,'YData',-5,'MarkerFaceColor',min(cR+[0.05 0.05 0.05],1));
        set(hBrkN,'XData',brk(1),'YData',brk(2));  set(hCldN,'XData',cld(1),'YData',cld(2));
        set(tLead,'Position',[xL-4 4.5 0]); set(tFoll,'Position',[xF-8 -4.5 0]); set(tRsu,'Position',[P.rsuX+3 -8 0]);
        set(tBrk,'Position',[brk(1)-18 brk(2)+6 0]); set(tCld,'Position',[cld(1)-4 cld(2)+6 0]);
        set(hSp,'XData',[xL cx NaN xF cx NaN P.rsuX cx NaN cld(1) cx],'YData',[0 30 NaN 0 30 NaN -5 30 NaN 50 30]);
        if useV2X
            setLink(hLnk.V2V,[xL xF],[0 0],t,S,'V2V',cLink.V2V);
            setLink(hLnk.V2I,[xL P.rsuX],[0 -5],t,S,'V2I',cLink.V2I);
            setLink(hLnk.I2V,[P.rsuX xF],[-5 0],t,S,'I2V',cLink.I2V);
            setLink(hLnk.V2N,[P.rsuX cld(1)],[-5 cld(2)],t,S,'V2N',cLink.V2N);
            if t >= P.stress(1) && t <= P.stress(2)
                set(hStress,'String',sprintf('NETWORK STRESS: drop %d%%, +%d ms',round(100*P.dropStress),round(1000*P.delayStress)));
            else
                set(hStress,'String','');
            end
        end
        title(axN,sprintf('PLOT 0: TOP-DOWN MQTT NETWORK GRAPH | broker msgs: %d | cloud rx: %d',S.msgCount,S.cloudRx), ...
              'Color','w','FontSize',10);
        if isempty(S.log), set(hLog,'String',{'MQTT LOG (topic, latency)'}); else, set(hLog,'String',[{'MQTT LOG (topic, latency)'}, S.log]); end

        % ----- HUD -----
        status = 'NOMINAL';  cStat = '\color[rgb]{0,1,0.5}';
        if collided,                                  status = 'IMPACT DETECTED';           cStat = '\color[rgb]{1,0.2,0.2}';
        elseif useV2X && t >= P.tBrake && t < tWarn && t < tReact, status = 'TX WARNING...'; cStat = '\color[rgb]{1,0.2,0.8}';
        elseif useV2X && t >= tWarn && t < tReact,    status = 'V2X RX: AUTO-BRAKE';        cStat = '\color[rgb]{1,1,0}';
        elseif t >= P.tBrake && t < tReact,           status = 'LEAD BRAKING (DRIVER LAG)'; cStat = '\color[rgb]{1,1,0}';
        elseif t >= tReact,                           status = 'BRAKING ENGAGED';           cStat = '\color[rgb]{1,0.5,0}';
        end
        set(hud,'String',{ sprintf('SYSTEM TIME   : %05.2f s',t), sprintf('LEAD VELOCITY : %05.1f m/s',vL), ...
            sprintf('FOLL VELOCITY : %05.1f m/s',vF), sprintf('REL. DISTANCE : %05.1f m',gapNow), ...
            sprintf('TTC           : %05.2f s',ttc), sprintf('V2V LATENCY   : %05.1f ms',S.latV2V), ...
            sprintf('RSU STATE     : %s',sR), netTag, ['SYS STATUS    : ' cStat status] });

        % ----- dashboard -----
        set(hLatV,'XData',dT(1:s),'YData',dLatV(1:s));  set(hLatI,'XData',dT(1:s),'YData',dLatI(1:s));
        set(hPdrV,'XData',dT(1:s),'YData',dPdrV(1:s));  set(hPdrI,'XData',dT(1:s),'YData',dPdrI(1:s));
        set(hGap, 'XData',dT(1:s),'YData',dGap(1:s));   set(hTtc, 'XData',dT(1:s),'YData',dTtc(1:s));

        fig.UserData = S;  drawnow;  pause(P.framePause);
        if P.saveVideo, writeVideo(vw, getframe(fig)); end
        if collided && t > tCol + 1.5, break; end
        if ~collided && vL == 0 && vF == 0 && t > P.tBrake + 2.5, pause(0.8); break; end
    end
    if P.saveVideo, close(vw); end

    S = fig.UserData;  n = S.nV2V;
    res.mode = modeStr;  res.collided = collided;  res.minGap = minGap;
    res.t = dT(1:s);  res.gap = dGap(1:s);  res.ttc = dTtc(1:s);  res.minTTC = min(dTtc(1:s));
    res.meanLat = mean(dLatV(1:s),'omitnan');
    if n > 0, res.pdr = 100*sum(S.v2vOk(1:n))/n; else, res.pdr = NaN; end
    res.msgs = S.msgCount;  res.real = net.real;
end

%% ======================= MQTT / network layer =======================
function net = netInit(P, fig)
    net.real = false;  net.fig = fig;  net.c = struct();
    setappdata(fig,'reg',cell(0,2));                       % registry used only by the loopback fallback
    if P.useMQTT
        try
            who = {'lead','foll','rsu','cloud','traf'};   % one MQTT client per IoT node
            tag = num2str(randi(1e6));
            for i = 1:numel(who)
                net.c.(who{i}) = mqttclient(P.broker,'Port',P.port,'ClientID',['v2x_' who{i} '_' tag]);
                if ~net.c.(who{i}).Connected, error('client %s not connected',who{i}); end
            end
            net.real = true;
            fprintf('MQTT: connected to %s:%d (real Mosquitto traffic)\n',P.broker,P.port);
        catch ME
            warning('MQTT unavailable (%s). Using in-process LOOPBACK broker (NOT real MQTT).',ME.message);
            net.c = struct();  net.real = false;
        end
    else
        fprintf('P.useMQTT = false: using in-process LOOPBACK broker.\n');
    end
end

function netSub(net, who, topic, cb, qos)
    if net.real
        subscribe(net.c.(who), topic, 'QualityOfService', qos, 'Callback', cb);
    else
        reg = getappdata(net.fig,'reg');  reg(end+1,:) = {topic, cb};  setappdata(net.fig,'reg',reg);
    end
end

function netWrite(net, who, topic, payload, qos)
    if net.real
        write(net.c.(who), topic, payload, 'QualityOfService', qos, 'Retain', false);
    else
        reg = getappdata(net.fig,'reg');
        for i = 1:size(reg,1)
            if topicMatch(reg{i,1}, topic), reg{i,2}(topic, payload); end
        end
    end
end

function S = pubMsg(fig, net, S, who, topic, payload, qos)
    fig.UserData = S;                 % sync so loopback callbacks do not overwrite local edits
    netWrite(net, who, topic, payload, qos);
    S = fig.UserData;
end

function tf = topicMatch(f, t)
    fp = strsplit(f,'/');  tp = strsplit(t,'/');  tf = false;
    for i = 1:numel(fp)
        if strcmp(fp{i},'#'), tf = true; return; end
        if i > numel(tp), return; end
        if ~strcmp(fp{i},'+') && ~strcmp(fp{i},tp{i}), return; end
    end
    tf = (numel(fp) == numel(tp));
end

function [drop, extra] = netStress(P, tSim)
    % Artificial network impairment applied to every received MQTT payload
    pDrop = P.dropBase;  extra = P.delayBase + P.jitter*rand;
    if tSim >= P.stress(1) && tSim <= P.stress(2)
        pDrop = P.dropStress;  extra = extra + P.delayStress*(0.5 + rand);
    end
    drop = rand < pDrop;
end

function onMsg(fig, P, role, tp, msg)
    % Subscriber callback: measure latency (real broker time + artificial delay), apply drops
    if ~isvalid(fig), return; end
    S = fig.UserData;
    S.msgCount = S.msgCount + 1;
    if strcmp(role,'load'), fig.UserData = S; return; end
    d = jsondecode(char(msg));
    [drop, extra] = netStress(P, d.tSim);
    lat = toc(S.T0) - d.tWall + extra;                         % seconds
    if strcmp(role,'v2n'), lat = lat + P.backhaul; end
    link = upper(role);                                        % V2V | V2I | I2V | V2N
    if drop
        S.lastDrop.(link) = d.tSim;
        S = addLog(S, sprintf('%5.2fs DROP %-24s',d.tSim,char(tp)));
    else
        S.lastOk.(link) = d.tSim;
        eff = d.tSim + lat;                                    % effective sim time of reception
        switch role
            case 'v2v'
                S.v2vOk(d.seq) = true;  S.latV2V = 1e3*lat;
                if d.brake == 1, S.warnV2V = min(S.warnV2V, eff); end
            case 'v2i'
                S.v2iOk(d.seq) = true;  S.q(end+1,:) = [eff d.tSim 0];
            case 'i2v'
                S.i2vOk(d.seq) = true;  S.latE2E = 1e3*(eff - d.tOrig);  S.warnI2V = min(S.warnI2V, eff);
            case 'v2n'
                S.cloudRx = S.cloudRx + 1;  S.latV2N = 1e3*lat;
        end
        S = addLog(S, sprintf('%5.2fs RX   %-24s %5.1f ms',d.tSim,char(tp),1e3*lat));
    end
    fig.UserData = S;
end

function S = addLog(S, line)
    S.log{end+1} = line;
    if numel(S.log) > 9, S.log(1) = []; end
end

function p = winPDR(tx, ok, n, t, W, g)
    p = NaN;  if n == 0, return; end
    tt = tx(1:n);  m = tt >= t-W-g & tt <= t-g;
    if any(m), o = ok(1:n); p = 100*mean(double(o(m))); end
end

function setLink(h, xs, ys, t, S, name, col)
    if t - S.lastOk.(name) < 0.3
        set(h,'XData',xs,'YData',ys,'Color',col,'LineWidth',3.5,'LineStyle','-');
    elseif t - S.lastDrop.(name) < 0.3
        set(h,'XData',xs,'YData',ys,'Color',[1 0.2 0.2],'LineWidth',2,'LineStyle','--');
    else
        set(h,'XData',xs,'YData',ys,'Color',[0.25 0.35 0.45],'LineWidth',1,'LineStyle',':');
    end
end

%% ======================= 3D models =======================
function c = makeNeonCar(ax, col, len, w)
    c.tf = hgtransform('Parent', ax);
    vShad = [-1 -1 0; 1 -1 0; 1 1 0; -1 1 0]/2;
    patch('Parent',c.tf,'Vertices',vShad .* [len+1.5, w+1.5, 0] + [0 0 0.01],'Faces',[1 2 3 4], ...
          'FaceColor','k','FaceAlpha',0.6,'EdgeColor','none');
    c.body = boxPatch(c.tf, 0,    0, 0.75, len,     w,     0.9, col);
    c.cab  = boxPatch(c.tf, -0.3, 0, 1.5,  len*0.4, w*0.8, 0.5, [0.05 0.05 0.08]); %#ok<NASGU>
    for sx = [-1.4 1.4]
        for sy = [-1 1]*(w/2)
            boxPatch(c.tf, sx, sy, 0.35, 0.9, 0.2, 0.7, col*1.5);
        end
    end
    c.hl = patch('Parent',c.tf,'Vertices',[len/2 0 0.75; len/2+15 -5 0; len/2+15 5 0],'Faces',[1 2 3], ...
                 'FaceColor',[1 1 0.5],'FaceAlpha',0.1,'EdgeColor','none'); %#ok<NASGU>
end

function triggerBrakes(c)
    set(c.body,'FaceVertexCData',shadeCol([1 0.1 0.1]));
end

function h = boxPatch(parent, cx, cy, cz, lx, ly, lz, col)
    V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1]/2;
    V = V .* [lx ly lz] + [cx cy cz];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    h = patch('Parent',parent,'Vertices',V,'Faces',F,'FaceColor','flat','FaceVertexCData',shadeCol(col), ...
              'EdgeColor',[0 0 0],'LineWidth',0.1,'FaceLighting','flat', ...
              'AmbientStrength',0.8,'DiffuseStrength',0.4,'SpecularStrength',0.2);
end

function C = shadeCol(col)
    s = [0.4; 1.0; 0.8; 0.6; 0.7; 0.5];
    C = min(s*col, 1);
end
