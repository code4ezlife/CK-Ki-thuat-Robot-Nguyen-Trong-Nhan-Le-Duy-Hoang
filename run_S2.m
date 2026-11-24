%% RUN_S2  Kịch bản S2 - Một người cắt ngang: so sánh DWA / DWA-KF / ORCA
%  Phần A: một lần chạy minh họa (quỹ đạo robot-người, v, w, d_edge, Kalman)
%  Phần B: chạy hàng loạt 50 hạt giống (101..150), in bảng thống kê và Wilcoxon
%  (Khảo sát độ nhạy kappa, sigma_z: chạy sensitivity_S2.m)
clear; clc; close all;
cfg    = config_params();
cfg.show_trajectory_figures = false; % thay cua so quy dao tinh bang video MP4
algos  = {'dwa', 'dwa_kf', 'orca'};
labels = {'DWA', 'DWA-KF', 'ORCA'};
quick  = false;      % true: chỉ chạy 5 hạt giống để thử nhanh
seeds  = cfg.seeds_eval;
if quick, seeds = seeds(1:5); end

%% ---------------- Phần A: một lần chạy minh họa -------------------------
scn = scenario_S2(seeds(1), cfg);
fprintf('Human appears at t = %.2f s, speed %.2f m/s (robot nominal arrival %.2f s, human %.2f s)\n', ...
    scn.humans.t_start, scn.humans.speed, scn.info.t_robot_nominal, scn.info.t_human_nominal);
R = cell(1, numel(algos));
for a = 1:numel(algos)
    R{a} = sim_main(scn, algos{a}, cfg);
    mm = compute_metrics(R{a}, scn, cfg);
    fprintf('[demo seed %d] %-7s success=%d collided=%d T=%.1fs L=%.2fm d_min=%.2fm near_miss=%d\n', ...
        seeds(1), labels{a}, mm.success, mm.collided, mm.T, mm.L, mm.d_min, mm.near_miss);
end
plot_results_S2(scn, R, labels, cfg);
export_demo_videos(scn, R, algos, cfg, 'S2');

%% ---------------- Phần B: chạy hàng loạt --------------------------------
nS = numel(seeds);
M  = [];
for s = 1:nS
    scn = scenario_S2(seeds(s), cfg);          % cùng hạt giống -> cùng điều kiện đầu và nhiễu
    for a = 1:numel(algos)
        res = sim_main(scn, algos{a}, cfg);
        M_s = compute_metrics(res, scn, cfg);
        if isempty(M), M = repmat(M_s, numel(algos), nS); end
        M(a, s) = M_s; %#ok<SAGROW>
    end
    fprintf('seed %d done (%d/%d)\n', seeds(s), s, nS);
end

summarize_results(M, labels, 'S2');
save('results_S2.mat', 'M', 'algos', 'labels', 'seeds', 'cfg');
fprintf('\nDa luu results_S2.mat\n');
