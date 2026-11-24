function plot_results_S3(scn, R, labels, cfg)
%PLOT_RESULTS_S3  Hình cho S3: dùng lại 3 hình của S2 (quỹ đạo, v/w/d_edge, Kalman) và
%   thêm hình vị trí NGANG y(t) để quan sát dao động / né sang bên của robot.
plot_results_S2(scn, R, labels, cfg);
col = {[0.85 0.33 0.10], [0 0.45 0.74], [0.47 0.67 0.19]};
figure('Name','S3 - Lateral motion','Color','w');
subplot(2,1,1); hold on; grid on;
for a = 1:numel(R), plot(R{a}.t, R{a}.y, 'Color',col{a}, 'LineWidth',1.5); end
nk = size(R{1}.hum_true, 1);  t = (0:nk-1)'*cfg.dt;
hy = reshape(R{1}.hum_true(:,1,2), nk, 1);
plot(t, hy, 'k--', 'LineWidth',1.2);
hw = cfg.s3_width/2;
plot(xlim, [6-hw 6-hw], 'k-');  plot(xlim, [6+hw 6+hw], 'k-');
ylabel('y [m]');  legend([labels, {'person'}], 'Location','best');
title('S3 - Lateral position (corridor walls in black)');
subplot(2,1,2); hold on; grid on;
for a = 1:numel(R), plot(R{a}.x, R{a}.y, 'Color',col{a}, 'LineWidth',1.5); end
ylabel('y [m]');  xlabel('x [m]');  title('Robot paths in the corridor');
end
