function plot_results_S5(scn, R, labels, cfg)
%PLOT_RESULTS_S5  Hình cho S5: (1) quỹ đạo, đường A* ban đầu và đường sau khi tính lại, vị trí
%   tính lại (x), đường đi của hai người; (2) v, w, khoảng cách tới người; (3) sai số dự đoán 2 s
%   của Kalman với người 2 (đỉnh sau thời điểm rẽ = "dự đoán sai").
col = {[0.85 0.33 0.10], [0 0.45 0.74], [0.47 0.67 0.19]};
nR = numel(R);  hc = {[0.93 0.69 0.13], [0.64 0.08 0.18]};

if ~isfield(cfg,'show_trajectory_figures') || cfg.show_trajectory_figures
% --- Hình 1 ------------------------------------------------------------------
figure('Name','S5 - Trajectories','Color','w','Position',[40 80 1350 450]);
for a = 1:nR
    r = R{a};  [nk, nH, ~] = size(r.hum_true);
    subplot(1, nR, a);  draw_map(scn.map);
    plot(r.path0(:,1), r.path0(:,2), 'k--', 'LineWidth',1);                    % A* ban đầu
    if r.n_replan > 0, pf = r.path_final; plot(pf(:,1), pf(:,2), ':', 'Color',col{a}, 'LineWidth',1.5); end
    for j = 1:nH
        hp = reshape(r.hum_true(:,j,:), nk, 2);
        act = (0:nk-1)'*cfg.dt >= scn.humans(j).t_start;
        plot(hp(act,1), hp(act,2), '-', 'Color',hc{j}, 'LineWidth',1.5);
        k0 = find(act, 1);  if ~isempty(k0), plot(hp(k0,1), hp(k0,2), 's', 'Color',hc{j}, 'MarkerFaceColor',hc{j}, 'MarkerSize',5); end
    end
    plot(r.x, r.y, '-', 'Color',col{a}, 'LineWidth',2);
    for tt = 2:2:floor(r.t(end))
        k = round(tt/cfg.dt) + 1;
        if k <= numel(r.x)
            plot(r.x(k), r.y(k), 'o', 'Color',col{a}, 'MarkerFaceColor',col{a}, 'MarkerSize',3);
            text(r.x(k)+0.12, r.y(k)-0.3, sprintf('%d', tt), 'FontSize',7);
        end
    end
    if ~isempty(r.replan_pos), plot(r.replan_pos(:,1), r.replan_pos(:,2), 'kx', 'MarkerSize',9, 'LineWidth',2); end
    plot(scn.start(1), scn.start(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
    plot(scn.goal(1),  scn.goal(2),  'rp', 'MarkerFaceColor','r', 'MarkerSize',13);
    if r.collided, tag = ' - COLLISION'; elseif ~r.success, tag = ' - TIMEOUT'; else, tag = ''; end
    title(sprintf('%s   d_{min,h} = %.2f m, replans = %d%s', labels{a}, min(r.d_human), r.n_replan, tag));
    xlabel('x [m]');  ylabel('y [m]');
end

end

% --- Hình 2 ------------------------------------------------------------------
figure('Name','S5 - Time histories','Color','w');
subplot(3,1,1); hold on; grid on; ylabel('v [m/s]');
for a = 1:nR, plot(R{a}.t, R{a}.v, 'Color',col{a}, 'LineWidth',1.5); end
legend(labels, 'Location','best');  title('S5 - Commands and clearance to the nearest person');
subplot(3,1,2); hold on; grid on; ylabel('\omega [rad/s]');
for a = 1:nR, plot(R{a}.t, R{a}.w, 'Color',col{a}, 'LineWidth',1.5); end
subplot(3,1,3); hold on; grid on; ylabel('d_{edge,human} [m]');  xlabel('t [s]');
for a = 1:nR, plot(R{a}.t, min(R{a}.d_human, 8), 'Color',col{a}, 'LineWidth',1.5); end
plot(xlim, [cfg.near_miss cfg.near_miss], 'k:');  plot(xlim, [0 0], 'r-');

% --- Hình 3: sai số dự đoán 2 s của Kalman với người 2 -----------------------
if numel(scn.humans) < 2, return; end
r = R{min(2, nR)};  [nk, ~, ~] = size(r.hum_true);  H = round(cfg.T_pred/cfg.dt);
est = reshape(r.hum_est(:,2,:), nk, 2);  ev = reshape(r.hum_estv(:,2,:), nk, 2);  tr = reshape(r.hum_true(:,2,:), nk, 2);
err = nan(nk, 1);
for k = 1:nk-H
    err(k) = norm(est(k,:) + cfg.T_pred*ev(k,:) - tr(k+H,:));      % dự đoán vận tốc không đổi tại t_k vs vị trí thật tại t_k + 2 s
end
t = (0:nk-1)'*cfg.dt;
figure('Name','S5 - Prediction error','Color','w'); hold on; grid on;
plot(t, err, '-', 'Color',[0.64 0.08 0.18], 'LineWidth',1.5);
yl = ylim;  plot([scn.info.t_turn scn.info.t_turn], yl, 'k--');
xlabel('t [s]');  ylabel('2 s prediction error of person 2 [m]');
title('Kalman prediction error; dashed = moment the person turns');
end
