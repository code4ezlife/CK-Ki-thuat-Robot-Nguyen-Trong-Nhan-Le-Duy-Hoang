%% RUN_S4  Kịch bản S4 - Nhiều người di chuyển: so sánh DWA / DWA-KF / ORCA
%  Không gian trống, 6 người đi cắt ngang bản đồ (0,8-1,4 m/s, hướng ngẫu nhiên).
%  Phần A: một lần chạy minh họa;  Phần B: 50 hạt giống (101..150), bảng thống kê + kiểm định.
%  Khảo sát phụ: scaling_S4.m (số người 1..10), sensitivity_S4.m (kappa, sigma_z).
clear; clc; close all;
cfg    = config_params();
cfg.show_trajectory_figures = false; % thay cua so quy dao tinh bang video MP4
% cfg.fallback = 'least_violation';   % tùy chọn (mặc định 'brake' = đúng báo cáo 3.3.4); xem README_S4.md
algos  = {'dwa', 'dwa_kf', 'orca'};
labels = {'DWA', 'DWA-KF', 'ORCA'};
quick  = false;      % true: chỉ chạy 5 hạt giống để thử nhanh
seeds  = cfg.seeds_eval;
if quick, seeds = seeds(1:5); end
fprintf('S4: %d people\n', cfg.s4_nhum);

%% ---------------- Phần A: một lần chạy minh họa -------------------------
scn = scenario_S4(seeds(1), cfg);
R = cell(1, numel(algos));
for a = 1:numel(algos)
    R{a} = sim_main(scn, algos{a}, cfg);
    mm = compute_metrics(R{a}, scn, cfg);
    fprintf('[demo seed %d] %-7s success=%d collided=%d T=%.1fs L=%.2fm d_min_h=%.2fm near_miss_h=%d t_comp=%.2fms\n', ...
        seeds(1), labels{a}, mm.success, mm.collided, mm.T, mm.L, mm.d_min_h, mm.near_miss_h, mm.t_comp_ms);
end
plot_results_S4(scn, R, labels, cfg);
export_demo_videos(scn, R, algos, cfg, 'S4');

%% ---------------- Phần B: chạy hàng loạt --------------------------------
nS = numel(seeds);
M  = [];
for s = 1:nS
    scn = scenario_S4(seeds(s), cfg);
    for a = 1:numel(algos)
        res = sim_main(scn, algos{a}, cfg);
        M_s = compute_metrics(res, scn, cfg);
        if isempty(M), M = repmat(M_s, numel(algos), nS); end
        M(a, s) = M_s; %#ok<SAGROW>
    end
    fprintf('seed %d done (%d/%d)\n', seeds(s), s, nS);
end
summarize_results(M, labels, 'S4');
save('results_S4.mat', 'M', 'algos', 'labels', 'seeds', 'cfg');
fprintf('\nDa luu results_S4.mat\n');
