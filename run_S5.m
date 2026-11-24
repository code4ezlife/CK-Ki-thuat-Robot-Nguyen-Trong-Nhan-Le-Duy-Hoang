%% RUN_S5  Kịch bản S5 - Đường bị chặn và đổi hướng đột ngột: DWA / DWA-KF / ORCA
%  Tường x = 6 có cửa 1,6 m + lối vòng phía trên. Người 1 dừng hẳn ở cửa (chặn đường);
%  người 2 đi thẳng rồi rẽ 90° vào đường robot ở giây thứ 6 (dự đoán sai).
%  Kiểm tra tính lại đường toàn cục (T1-T3) và độ bền khi dự đoán sai.
%  Phần A: một lần chạy minh họa;  Phần B: 50 hạt giống (101..150), bảng thống kê + kiểm định.
clear; clc; close all;
cfg    = config_params();
cfg.show_trajectory_figures = false; % thay cua so quy dao tinh bang video MP4
cfg.fallback = 'least_violation';   % khác báo cáo 3.3.4 ('brake'); chọn trên hạt giống 1-20, xem README_S5.md
% cfg.s5_human2 = false;            % tùy chọn đối chứng: bỏ người 2 (chỉ người 1 chặn cửa)
algos  = {'dwa', 'dwa_kf', 'orca'};
labels = {'DWA', 'DWA-KF', 'ORCA'};
quick  = false;      % true: chỉ chạy 5 hạt giống để thử nhanh
seeds  = cfg.seeds_eval;
if quick, seeds = seeds(1:5); end

%% ---------------- Phần A: một lần chạy minh họa -------------------------
scn = scenario_S5(seeds(1), cfg);
R = cell(1, numel(algos));
for a = 1:numel(algos)
    R{a} = sim_main(scn, algos{a}, cfg);
    mm = compute_metrics(R{a}, scn, cfg);
    fprintf('[demo seed %d] %-7s success=%d collided=%d T=%.1fs L=%.2fm d_min_h=%.2fm replans=%d first=%.1fs\n', ...
        seeds(1), labels{a}, mm.success, mm.collided, mm.T, mm.L, mm.d_min_h, mm.n_replan, mm.t_replan1);
    if ~isempty(R{a}.replan_log), fprintf('      replans: %s\n', strjoin(R{a}.replan_log, ', ')); end
end
plot_results_S5(scn, R, labels, cfg);
export_demo_videos(scn, R, algos, cfg, 'S5');

%% ---------------- Phần B: chạy hàng loạt --------------------------------
nS = numel(seeds);
M  = [];
for s = 1:nS
    scn = scenario_S5(seeds(s), cfg);
    for a = 1:numel(algos)
        res = sim_main(scn, algos{a}, cfg);
        M_s = compute_metrics(res, scn, cfg);
        if isempty(M), M = repmat(M_s, numel(algos), nS); end
        M(a, s) = M_s; %#ok<SAGROW>
    end
    fprintf('seed %d done (%d/%d)\n', seeds(s), s, nS);
end
summarize_results(M, labels, 'S5');
save('results_S5.mat', 'M', 'algos', 'labels', 'seeds', 'cfg');
fprintf('\nDa luu results_S5.mat\n');
