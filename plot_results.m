function plot_results(scn, R, labels, cfg)
%PLOT_RESULTS  So sánh quỹ đạo, vận tốc và khoảng cách an toàn của các thuật toán.
col = {[0.85 0.33 0.10], [0 0.45 0.74], [0.47 0.67 0.19]};

if ~isfield(cfg,'show_trajectory_figures') || cfg.show_trajectory_figures
% --- Hình 1: quỹ đạo trên bản đồ ----------------------------------------
figure('Name','S1 - Trajectories','Color','w');
draw_map(scn.map);
p0 = R{1}.path0;
plot(p0(:,1), p0(:,2), 'k--', 'LineWidth',1.2, 'DisplayName','A* path');
h = gobjects(1, numel(R));
for a = 1:numel(R)
    h(a) = plot(R{a}.x, R{a}.y, '-', 'Color',col{a}, 'LineWidth',2, 'DisplayName',labels{a});
end
plot(scn.start(1), scn.start(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',9, 'HandleVisibility','off');
plot(scn.goal(1),  scn.goal(2),  'rp', 'MarkerFaceColor','r', 'MarkerSize',14, 'HandleVisibility','off');
text(scn.start(1)+0.2, scn.start(2)-0.4, 'S');  text(scn.goal(1)-0.5, scn.goal(2)+0.4, 'G');
title('S1 - Static obstacles (warehouse)');  xlabel('x [m]');  ylabel('y [m]');
legend('Location','southoutside','Orientation','horizontal');

end

% --- Hình 2: v, w, d_edge theo thời gian -------------------------------
figure('Name','S1 - Time histories','Color','w');
subplot(3,1,1); hold on; grid on; ylabel('v [m/s]');
for a = 1:numel(R), plot(R{a}.t, R{a}.v, 'Color',col{a}, 'LineWidth',1.5); end
legend(labels, 'Location','best');  title('Commands and clearance');
subplot(3,1,2); hold on; grid on; ylabel('\omega [rad/s]');
for a = 1:numel(R), plot(R{a}.t, R{a}.w, 'Color',col{a}, 'LineWidth',1.5); end
subplot(3,1,3); hold on; grid on; ylabel('d_{edge} [m]');  xlabel('t [s]');
for a = 1:numel(R), plot(R{a}.t, R{a}.d_edge, 'Color',col{a}, 'LineWidth',1.5); end
plot(xlim, [cfg.near_miss cfg.near_miss], 'k:');
end
