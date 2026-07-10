clear; clc;

params;
mdl = 'CP27E_TC';
if ~bdIsLoaded(mdl); build_cp27e_tc(); end

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

%% ---- headline metrics ----
i75 = find(dist >= 75, 1);
if isempty(i75)
    fprintf('run_sim: reached %.1f m by t=%.2f s\n', dist(end), t(end));
else
    fprintf('run_sim: 0-75 m in %.2f s (v=%.1f m/s)\n', t(i75), vx(i75));
end
fprintf('  peak vx=%.1f m/s   peak |ax|=%.1f   peak |ay|=%.1f m/s^2\n', ...
    max(vx), max(abs(ax)), max(abs(ay)));
fprintf('  peak combined mu_util [FL FR RL RR] = [%.2f %.2f %.2f %.2f]\n', max(muu));

figure('Name','CP27E 4-wheel TC (full envelope)','Color','w', ...
    'Position',[60 60 1280 760]);
tl = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

nexttile; plot(t,slp,'LineWidth',1); hold on; pl();
yline( P.slip_tgt,'k--'); yline(-P.slip_tgt,'k--'); grid on
title('slip ratio / wheel  (\pm target)'); xlabel t; ylabel \sigma
legend(lbl,'Location','best'); ylim([-0.5 0.5]);

nexttile; plot(t,Tc,'LineWidth',1); hold on; pl();
plot(t,Tmd,'--','LineWidth',0.6); grid on
title('motor torque cmd + grip ceiling'); xlabel t; ylabel 'N\cdotm'
legend([lbl {'T_{max,drv}'}],'Location','best');

nexttile; yyaxis left; plot(t,vx,'LineWidth',1.3); ylabel 'v_x [m/s]'; hold on
yyaxis right; plot(t,ax,'-',t,ay,'--','LineWidth',1); ylabel 'accel [m/s^2]'; pl()
grid on; title('speed, a_x, a_y'); xlabel t; legend({'v_x','a_x','a_y'},'Location','best');

nexttile; plot(t,Fz,'LineWidth',1); hold on; pl(); grid on
title('normal load / wheel  (lateral transfer in corner)'); xlabel t; ylabel 'F_z [N]'
legend(lbl,'Location','best');

nexttile; plot(t,muu,'LineWidth',1); hold on; pl();
yline(1,'k--','limit'); grid on
title('COMBINED friction utilization / wheel'); xlabel t; ylabel '|F_x| / F_{x,cap}'
legend(lbl,'Location','best'); ylim([0 1.3]);

nexttile; yyaxis left; plot(t,Trq,'LineWidth',1.3); ylabel 'T_{req} [N\cdotm]'; hold on
yyaxis right; plot(t,ay,'LineWidth',1); ylabel 'a_y [m/s^2]'; pl()
grid on; title('scenario: torque request & lateral'); xlabel t

title(tl, sprintf('CP27E 4-wheel hub-motor TC  |  gear=%.1f  T_{pk}=%.0f Nm  %s', ...
    P.gear, P.Tmot_pk, 'launch \rightarrow corner'));

%% ---- normalize any per-wheel log to N x 4 ----
function M = as4(D)
    D = squeeze(D);
    if size(D,1)==4 && size(D,2)~=4, D = D.'; end
    M = D;
end