function plot_results_S2(scn, R, labels, cfg)
%PLOT_RESULTS_S2  Hình cho S2 (và S3, dùng chung; tên kịch bản lấy từ scn.name): (1) quỹ đạo robot-người có mốc thời gian, (2) v, w, d_edge,
%   (3) chất lượng bám của bộ lọc Kalman.
col = {[0.85 0.33 0.10], [0 0.45 0.74], [0.47 0.67 0.19]};
nR = numel(R);  th = linspace(0, 2*pi, 40);

if ~isfield(cfg,'show_trajectory_figures') || cfg.show_trajectory_figures
% --- Hình 1: quỹ đạo + vị trí hai bên tại cùng thời điểm -----------------
figure('Name',[scn.name ' - Trajectories'],'Color','w','Position',[40 80 1350 450]);
for a = 1:nR
    r = R{a};  nk = size(r.hum_true, 1);
    subplot(1, nR, a);  draw_map(scn.map);
    hp = reshape(r.hum_true(:,1,:), nk, 2);
    plot(hp(:,1), hp(:,2), '-', 'Color',[0.93 0.69 0.13], 'LineWidth',1.5);
    plot(r.x, r.y, '-', 'Color',col{a}, 'LineWidth',2);
    for tt = 1:1:floor(r.t(end))                       % mốc mỗi 1 s
        k = round(tt/cfg.dt) + 1;
        if k <= nk && k <= numel(r.x)
            plot(r.x(k), r.y(k), 'o', 'Color',col{a}, 'MarkerFaceColor',col{a}, 'MarkerSize',4);
            plot(hp(k,1), hp(k,2), 'o', 'Color',[0.93 0.69 0.13], 'MarkerFaceColor',[0.93 0.69 0.13], 'MarkerSize',4);
            if mod(tt,2) == 0, text(r.x(k)+0.1, r.y(k)-0.35, sprintf('%d s', tt), 'FontSize',7); end
        end
    end
    [dm, ic] = min(r.d_human);  kc = min(ic, nk);   % gần NGƯỜI nhất       % thời điểm gần nhất
    rectangle('Position',[r.x(ic)-cfg.r_robot r.y(ic)-cfg.r_robot 2*cfg.r_robot 2*cfg.r_robot], ...
              'Curvature',[1 1], 'EdgeColor',col{a}, 'LineWidth',1.5);
    rectangle('Position',[hp(kc,1)-cfg.r_human hp(kc,2)-cfg.r_human 2*cfg.r_human 2*cfg.r_human], ...
              'Curvature',[1 1], 'EdgeColor',[0.93 0.69 0.13], 'LineWidth',1.5);
    plot(scn.start(1), scn.start(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
    plot(scn.goal(1),  scn.goal(2),  'rp', 'MarkerFaceColor','r', 'MarkerSize',13);
    if r.collided, tag = ' - COLLISION'; else, tag = ''; end
    title(sprintf('%s   d_{min,human} = %.2f m%s', labels{a}, dm, tag));
    xlabel('x [m]');  ylabel('y [m]');
end

end

% --- Hình 2: v, w, d_edge theo thời gian ---------------------------------
figure('Name',[scn.name ' - Time histories'],'Color','w');
subplot(3,1,1); hold on; grid on; ylabel('v [m/s]');
for a = 1:nR, plot(R{a}.t, R{a}.v, 'Color',col{a}, 'LineWidth',1.5); end
legend(labels, 'Location','best');  title([scn.name ' - Commands and clearance']);
subplot(3,1,2); hold on; grid on; ylabel('\omega [rad/s]');
for a = 1:nR, plot(R{a}.t, R{a}.w, 'Color',col{a}, 'LineWidth',1.5); end
subplot(3,1,3); hold on; grid on; ylabel('d_{edge} [m]');  xlabel('t [s]');
for a = 1:nR, plot(R{a}.t, R{a}.d_edge, 'Color',col{a}, 'LineWidth',1.5); end
plot(xlim, [cfg.near_miss cfg.near_miss], 'k:');  plot(xlim, [0 0], 'r-');

% --- Hình 3: bộ lọc Kalman (giống nhau ở cả ba thuật toán -> lấy lần chạy 2) ---
r = R{min(2, nR)};  nk = size(r.hum_true, 1);  t = (0:nk-1)'*cfg.dt;
tp = reshape(r.hum_true(:,1,:), nk, 2);  ep = reshape(r.hum_est(:,1,:),  nk, 2);
mp = reshape(r.hum_meas(:,1,:), nk, 2);  ev = reshape(r.hum_estv(:,1,:), nk, 2);
% vận tốc thật: sai phân vị trí thật
tv = [diff(tp)/cfg.dt; nan(1,2)];
figure('Name',[scn.name ' - Kalman tracking'],'Color','w');
subplot(3,1,1); hold on; grid on;
plot(t, tp(:,2), 'k-', 'LineWidth',1.5);  plot(t, mp(:,2), '.', 'Color',[0.7 0.7 0.7]);
plot(t, ep(:,2), '-', 'Color',[0 0.45 0.74], 'LineWidth',1.5);
ylabel('y of human [m]');  legend('true','measured','KF estimate','Location','best');
title('Kalman filter tracking of the person');
subplot(3,1,2); hold on; grid on;
plot(t, sqrt(sum((mp - tp).^2, 2)), '.', 'Color',[0.7 0.7 0.7]);
plot(t, sqrt(sum((ep - tp).^2, 2)), '-', 'Color',[0 0.45 0.74], 'LineWidth',1.5);
ylabel('position error [m]');  legend('raw measurement','KF estimate','Location','best');
subplot(3,1,3); hold on; grid on;
plot(t, sqrt(sum(tv.^2, 2)), 'k-', 'LineWidth',1.5);
plot(t, sqrt(sum(ev.^2, 2)), '-', 'Color',[0 0.45 0.74], 'LineWidth',1.5);
ylabel('speed [m/s]');  xlabel('t [s]');  legend('true','KF estimate','Location','best');
end
