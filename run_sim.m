clear; clc;

params;
mdl = 'CP27E_TC';
if ~bdIsLoaded(mdl); open_system(mdl); end

set_param(mdl,'SolverType','Variable-step','SolverName',P.solver, ...
    'StopTime',num2str(P.tEnd),'MaxStep',num2str(P.maxStep), ...
    'ReturnWorkspaceOutputs','on');
out = sim(mdl);
ds  = out.logsout;
gv  = @(name) ds.get(name).Values;

%% ---- pull logs ----
t    = gv('vx').Time;                sc  = gv('sched').Data(:);
vx   = gv('vx').Data(:);             ax  = gv('ax').Data(:);
ay   = gv('ay').Data(:);            Trq  = gv('Treq').Data(:);
w    = as4(gv('w').Data);           slp  = as4(gv('slip').Data);
Tc   = as4(gv('Tcmd').Data);         Fz  = as4(gv('Fz').Data);
muu  = as4(gv('mu_util').Data);     Fxc  = as4(gv('Fx_cap').Data);
Tmd  = as4(gv('Tmax_drv').Data);

dist = cumtrapz(t, vx);
lbl  = {'FL','FR','RL','RR'};
pb   = P.t1;                                    % phase boundaries
pl   = @() arrayfun(@(x) xline(x,'k:','HandleVisibility','off'), pb, 'UniformOutput', false);

%% ---- headline metrics: interpolated 75 m crossing ----
i75 = find(dist >= 75, 1);
if isempty(i75)
    t75 = NaN;
    fprintf('run_sim: reached only %.1f m by t=%.2f s (never hit 75 m)\n', dist(end), t(end));
else
    d0 = dist(i75-1); d1 = dist(i75);
    fr = (75 - d0)/(d1 - d0);
    t75 = t(i75-1) + fr*(t(i75) - t(i75-1));
    v75 = vx(i75-1) + fr*(vx(i75) - vx(i75-1));
    fprintf('run_sim: 0-75 m in %.3f s (v=%.1f m/s)\n', t75, v75);
end

%% ---- front/rear averages ----
slpF = mean(slp(:,1:2),2);   slpR = mean(slp(:,3:4),2);
TcF  = mean(Tc(:,1:2),2);    TcR  = mean(Tc(:,3:4),2);
FzF  = mean(Fz(:,1:2),2);    FzR  = mean(Fz(:,3:4),2);

% motor torque-speed envelope (max possible torque at each instant)
wmot = max(abs(vx)/P.Rw*P.gear, 1);
Tenv = min(P.Tmot_pk, min(P.Pmot_pk, P.Pcap_veh/4)./wmot);

xmax = min(t(end), t75*1.05);          % focus all axes on the accel run
if isnan(xmax), xmax = t(end); end

%% ---- plots: 0-75 m accel focus ----
fig = figure('Name','CP27E accel 0-75m','Color','w','Position',[60 60 1280 760]);
tl  = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

nexttile; plot(t,slpF,'b',t,slpR,'r','LineWidth',1.2); hold on
yline(P.slip_tgt,'k--','target'); grid on; xlim([0 xmax])
title('slip ratio (axle avg)'); xlabel('t [s]'); ylabel('\sigma')
legend({'front','rear'},'Location','best'); ylim([-0.05 0.5]);

nexttile; plot(t,TcF,'b',t,TcR,'r','LineWidth',1.2); hold on
plot(t,Tenv,'k--','LineWidth',1); grid on; xlim([0 xmax])
title('motor torque (axle avg)'); xlabel('t [s]'); ylabel('N\cdotm')
legend({'front','rear','T_{max} envelope'},'Location','best');

nexttile; plot(t,FzF,'b',t,FzR,'r','LineWidth',1.2); grid on; xlim([0 xmax])
title('normal load (axle avg)'); xlabel('t [s]'); ylabel('F_z [N]')
legend({'front','rear'},'Location','best');

nexttile; plot(t,ax,'LineWidth',1.2); hold on; grid on; xlim([0 xmax])
if ~isnan(t75), xline(t75,'k:','75 m'); end
title('longitudinal accel'); xlabel('t [s]'); ylabel('a_x [m/s^2]');

nexttile; plot(t,dist,'LineWidth',1.2); hold on; grid on; xlim([0 xmax])
yline(75,'k:'); if ~isnan(t75), xline(t75,'k:','75 m'); end
title('position'); xlabel('t [s]'); ylabel('x [m]');

title(tl, sprintf('CP27E accel  |  0-75 m = %.3f s  |  gear=%.1f  T_{pk}=%.0f Nm  P=%.0f kW/motor', ...
    t75, P.gear, P.Tmot_pk, min(P.Pmot_pk,P.Pcap_veh/4)/1e3));

%% ---- save PNG ----
outdir = 'plots';
if ~exist(outdir,'dir'); mkdir(outdir); end
fname = fullfile(outdir, sprintf('accel_%s.png', datestr(now,'yyyy-mm-dd_HHMMSS')));
exportgraphics(fig, fname, 'Resolution', 200);
fprintf('saved %s\n', fname);

%% ---- normalize any per-wheel log to N x 4 ----
function M = as4(D)
D = squeeze(D);
if size(D,1)==4 && size(D,2)~=4, D = D.'; end
M = D;
end