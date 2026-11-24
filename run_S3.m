%% RUN_S3  Kịch bản S3 - Ngược chiều trong hành lang hẹp: so sánh DWA / DWA-KF / ORCA
%  Hành lang dài 12 m; người đi ngược chiều từ (10;6) với ~1,0 m/s, không nhường.
%  CẤU HÌNH S3 DÙNG Ở ĐÂY (khác Bảng 3.1 / mục 3.3.4, phải ghi rõ trong báo cáo):
%     - bề rộng hành lang 4,0 m (báo cáo: 2 m) - ở 2 m không còn quỹ đạo hợp lệ để vượt qua người
%     - cơ chế dự phòng 'least_violation' (báo cáo: phanh + quay tại chỗ)
%  Chọn trên tập hiệu chỉnh (hạt giống 1-20); kết quả bên dưới đánh giá trên 101-150.
%  Phần A: một lần chạy minh họa (quỹ đạo, v/w/d_edge, Kalman, vị trí ngang)
%  Phần B: 50 hạt giống (101..150), bảng thống kê và kiểm định theo cặp
%
%  CÔNG TẮC (xem README_S3.md để hiểu vì sao cần):
%     cfg.s3_width : bề rộng hành lang [m]  (Bảng 3.1: 2.0)
%     cfg.fallback : 'brake' (báo cáo 3.3.4) hoặc 'least_violation' (né tiếp khi hết quỹ đạo hợp lệ)
clear; clc; close all;
cfg    = config_params();
cfg.show_trajectory_figures = false; % thay cua so quy dao tinh bang video MP4
cfg.s3_width = 4.0;                 % đặt 2.0 để tái hiện đúng Bảng 3.1 (kết quả: ~100 % va chạm)
cfg.fallback = 'least_violation';   % đặt 'brake' để dùng đúng mô tả báo cáo
algos  = {'dwa', 'dwa_kf', 'orca'};
labels = {'DWA', 'DWA-KF', 'ORCA'};
quick  = false;      % true: chỉ chạy 5 hạt giống để thử nhanh
seeds  = cfg.seeds_eval;
if quick, seeds = seeds(1:5); end
fprintf('S3: corridor width = %.1f m, fallback = %s\n', cfg.s3_width, cfg.fallback);

%% ---------------- Phần A: một lần chạy minh họa -------------------------
scn = scenario_S3(seeds(1), cfg);
fprintf('Person appears at t = %.2f s, speed %.2f m/s\n', scn.info.t_start, scn.info.speed);
R = cell(1, numel(algos));
for a = 1:numel(algos)
    R{a} = sim_main(scn, algos{a}, cfg);
    mm = compute_metrics(R{a}, scn, cfg);
    fprintf('[demo seed %d] %-7s success=%d collided=%d T=%.1fs L=%.2fm d_min_h=%.2fm reversals=%d\n', ...
        seeds(1), labels{a}, mm.success, mm.collided, mm.T, mm.L, mm.d_min_h, mm.reversals);
end
plot_results_S3(scn, R, labels, cfg);
export_demo_videos(scn, R, algos, cfg, 'S3');

%% ---------------- Phần B: chạy hàng loạt --------------------------------
nS = numel(seeds);
M  = [];
for s = 1:nS
    scn = scenario_S3(seeds(s), cfg);
    for a = 1:numel(algos)
        res = sim_main(scn, algos{a}, cfg);
        M_s = compute_metrics(res, scn, cfg);
        if isempty(M), M = repmat(M_s, numel(algos), nS); end
        M(a, s) = M_s; %#ok<SAGROW>
    end
    fprintf('seed %d done (%d/%d)\n', seeds(s), s, nS);
end
summarize_results(M, labels, 'S3');
save('results_S3.mat', 'M', 'algos', 'labels', 'seeds', 'cfg');
fprintf('\nDa luu results_S3.mat\n');
