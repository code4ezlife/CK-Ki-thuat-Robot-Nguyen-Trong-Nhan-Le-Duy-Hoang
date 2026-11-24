function plot_results_S4(scn, R, labels, cfg)
%PLOT_RESULTS_S4  Hình cho S4 (nhiều người): (1) quỹ đạo robot + đường đi của mọi người,
%   (2) v, w, khoảng cách tới người gần nhất, (3) sai số bộ lọc Kalman trung bình, (4) thời gian tính.
col = {[0.85 0.33 0.10], [0 0.45 0.74], [0.47 0.67 0.19]};
nR = numel(R);  th = linspace(0, 2*pi, 40);  orange = [0.93 0.69 0.13];

if ~isfield(cfg,'show_trajectory_figures') || cfg.show_trajectory_figures
% --- Hình 1 ----------------------------------------------------------------
figure('Name',[scn.name ' - Trajectories'],'Color','w','Position',[40 80 1350 450]);
for a = 1:nR
    r = R{a};  [nk, nH, ~] = size(r.hum_true);
    subplot(1, nR, a);  draw_map(scn.map);
    for j = 1:nH
        hp = reshape(r.hum_true(:,j,:), nk, 2);
        act = (0:nk-1)'*cfg.dt >= scn.humans(j).t_start;
        plot(hp(act,1), hp(act,2), '-', 'Color',orange, 'LineWidth',1.2);
        k0 = find(act, 1);  if ~isempty(k0), plot(hp(k0,1), hp(k0,2), 's', 'Color',orange, 'MarkerFaceColor',orange, 'MarkerSize',4); end
    end
    plot(r.x, r.y, '-', 'Color',col{a}, 'LineWidth',2);
    for tt = 2:2:floor(r.t(end))
        k = round(tt/cfg.dt) + 1;
        if k <= numel(r.x)
            plot(r.x(k), r.y(k), 'o', 'Color',col{a}, 'MarkerFaceColor',col{a}, 'MarkerSize',4);
            text(r.x(k)+0.15, r.y(k)-0.3, sprintf('%d s', tt), 'FontSize',7);
        end
    end
    [dm, ic] = min(r.d_human);  kc = min(ic, nk);          % lúc gần người nhất
    dd = zeros(nH,1);
    for j = 1:nH, dd(j) = norm(squeeze(r.hum_true(kc,j,:))' - [r.x(ic) r.y(ic)]); end
    [~, jn] = min(dd);  hn = squeeze(r.hum_true(kc,jn,:))';
    rectangle('Position',[r.x(ic)-cfg.r_robot r.y(ic)-cfg.r_robot 2*cfg.r_robot 2*cfg.r_robot], 'Curvature',[1 1], 'EdgeColor',col{a}, 'LineWidth',1.5);
    rectangle('Position',[hn(1)-cfg.r_human hn(2)-cfg.r_human 2*cfg.r_human 2*cfg.r_human], 'Curvature',[1 1], 'EdgeColor',orange, 'LineWidth',1.5);
    plot(scn.start(1), scn.start(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
    plot(scn.goal(1),  scn.goal(2),  'rp', 'MarkerFaceColor','r', 'MarkerSize',13);
    if r.collided, tag = ' - COLLISION'; else, tag = ''; end
    title(sprintf('%s   d_{min,human} = %.2f m%s', labels{a}, dm, tag));
    xlabel('x [m]');  ylabel('y [m]');  xlim([-1 13]);  ylim([-1 13]);
end

end

% --- Hình 2 ----------------------------------------------------------------
figure('Name',[scn.name ' - Time histories'],'Color','w');
subplot(3,1,1); hold on; grid on; ylabel('v [m/s]');
for a = 1:nR, plot(R{a}.t, R{a}.v, 'Color',col{a}, 'LineWidth',1.5); end
legend(labels, 'Location','best');  title([scn.name ' - Commands and clearance to the nearest person']);
subplot(3,1,2); hold on; grid on; ylabel('\omega [rad/s]');
for a = 1:nR, plot(R{a}.t, R{a}.w, 'Color',col{a}, 'LineWidth',1.5); end
subplot(3,1,3); hold on; grid on; ylabel('d_{edge,human} [m]');  xlabel('t [s]');
for a = 1:nR, plot(R{a}.t, min(R{a}.d_human, 8), 'Color',col{a}, 'LineWidth',1.5); end
plot(xlim, [cfg.near_miss cfg.near_miss], 'k:');  plot(xlim, [0 0], 'r-');

% --- Hình 3: sai số Kalman trung bình trên các người đang được theo dõi -----
r = R{min(2, nR)};  [nk, nH, ~] = size(r.hum_true);  t = (0:nk-1)'*cfg.dt;
ee = sqrt(sum((r.hum_est  - r.hum_true).^2, 3));   % nk x nH (NaN nếu chưa theo dõi)
em = sqrt(sum((r.hum_meas - r.hum_true).^2, 3));
fe = ee; fe(isnan(fe)) = 0;  ce = sum(~isnan(ee), 2);  me = sum(fe, 2) ./ max(ce, 1);  me(ce == 0) = NaN;
fm = em; fm(isnan(fm)) = 0;  cm = sum(~isnan(em), 2);  mm = sum(fm, 2) ./ max(cm, 1);  mm(cm == 0) = NaN;
figure('Name',[scn.name ' - Kalman tracking'],'Color','w'); hold on; grid on;
plot(t, mm, '.', 'Color',[0.6 0.6 0.6]);  plot(t, me, '-', 'Color',[0 0.45 0.74], 'LineWidth',1.5);
xlabel('t [s]');  ylabel('mean position error [m]');  legend('raw measurement','KF estimate','Location','best');
title('Kalman filter: mean error over tracked people');

% --- Hình 4: thời gian tính toán -------------------------------------------
figure('Name',[scn.name ' - Computation time'],'Color','w');
tc = cellfun(@(x) 1000*mean(x.tcomp), R);
bar(tc);  set(gca, 'XTickLabel', labels);  ylabel('mean computation time per cycle [ms]');  grid on;
title(sprintf('Planner computation time with %d people (dt = %.0f ms)', numel(scn.humans), 1000*cfg.dt));
end
