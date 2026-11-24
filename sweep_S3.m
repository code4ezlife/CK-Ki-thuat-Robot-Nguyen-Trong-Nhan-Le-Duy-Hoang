%% SWEEP_S3  Khảo sát "S3 có giải được không": bề rộng hành lang x cơ chế dự phòng.
%  Chạy trên tập hạt giống hiệu chỉnh 1-20 (không dùng tập đánh giá 101-150).
%  Cho biết tỉ lệ va chạm của từng thuật toán để chọn cấu hình S3 hợp lý.
clear; clc;
base   = config_params();
seeds  = base.seeds_tune;
algos  = {'dwa','dwa_kf','orca'};
widths = [2.0 3.2 3.6 4.0 4.5];
fbs    = {'brake','least_violation'};
fprintf('%-18s %-7s %-9s %-9s %-9s\n', 'fallback', 'width', 'DWA', 'DWA-KF', 'ORCA');
fprintf('%-18s %-7s %-9s %-9s %-9s\n', '', '[m]', 'collis%', 'collis%', 'collis%');
for f = 1:numel(fbs)
    for W = widths
        cfg = base;  cfg.s3_width = W;  cfg.fallback = fbs{f};
        c = zeros(1, 3);
        for a = 1:3
            for sd = seeds
                scn = scenario_S3(sd, cfg);
                c(a) = c(a) + sim_main(scn, algos{a}, cfg).collided;
            end
        end
        fprintf('%-18s %-7.1f %-9.0f %-9.0f %-9.0f\n', fbs{f}, W, 100*c/numel(seeds));
    end
end
