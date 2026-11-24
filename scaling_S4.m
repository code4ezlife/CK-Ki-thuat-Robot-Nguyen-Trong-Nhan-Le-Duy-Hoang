%% SCALING_S4  Khả năng mở rộng theo số người (mục tiêu của S4): tỉ lệ va chạm và thời gian tính
%  theo số người 1, 2, 4, 6, 8, 10. Dùng 10 hạt giống đầu của tập đánh giá cho mỗi mức.
clear; clc; close all;
base   = config_params();
algos  = {'dwa','dwa_kf','orca'};  labels = {'DWA','DWA-KF','ORCA'};
nh     = [1 2 4 6 8 10];
seeds  = base.seeds_eval(1:10);
coll = zeros(numel(nh), 3);  tcomp = zeros(numel(nh), 3);  dmh = nan(numel(nh), 3);
for i = 1:numel(nh)
    cfg = base;  cfg.s4_nhum = nh(i);
    for a = 1:3
        c = 0;  tc = [];  dm = [];
        for sd = seeds
            scn = scenario_S4(sd, cfg);
            m = compute_metrics(sim_main(scn, algos{a}, cfg), scn, cfg);
            c = c + m.collided;  tc(end+1) = m.t_comp_ms;  dm(end+1) = m.d_min_h; %#ok<SAGROW>
        end
        coll(i,a) = 100*c/numel(seeds);  tcomp(i,a) = mean(tc);  dmh(i,a) = mean(dm);
    end
    fprintf('n=%2d  collision%%: DWA %3.0f | DWA-KF %3.0f | ORCA %3.0f   t_comp[ms]: %.2f | %.2f | %.2f\n', ...
        nh(i), coll(i,:), tcomp(i,:));
end
save('results_S4_scaling.mat', 'nh', 'coll', 'tcomp', 'dmh', 'seeds');
figure('Name','S4 - Scaling','Color','w');
subplot(1,3,1); plot(nh, coll,  'o-', 'LineWidth',1.5); grid on; xlabel('number of people'); ylabel('collision rate [%]'); legend(labels,'Location','best');
subplot(1,3,2); plot(nh, dmh,   'o-', 'LineWidth',1.5); grid on; xlabel('number of people'); ylabel('mean d_{min,human} [m]');
subplot(1,3,3); plot(nh, tcomp, 'o-', 'LineWidth',1.5); grid on; xlabel('number of people'); ylabel('computation time [ms/cycle]');
