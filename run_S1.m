%% RUN_S1  Kịch bản S1 - Vật cản tĩnh: so sánh DWA / DWA-KF / ORCA
%  Phần A: một lần chạy minh họa (vẽ quỹ đạo, v, w, d_edge)
%  Phần B: chạy hàng loạt 50 hạt giống (101..150) và in bảng thống kê
clear; clc; close all;
cfg    = config_params();
cfg.show_trajectory_figures = false; % thay cua so quy dao tinh bang video MP4
algos  = {'dwa', 'dwa_kf', 'orca'};
labels = {'DWA', 'DWA-KF', 'ORCA'};
quick  = false;      % true: chỉ chạy 5 hạt giống để thử nhanh
seeds  = cfg.seeds_eval;
if quick, seeds = seeds(1:5); end

%% ---------------- Phần A: một lần chạy minh họa -------------------------
scn = scenario_S1(seeds(1));
R = cell(1, numel(algos));
for a = 1:numel(algos)
    R{a} = sim_main(scn, algos{a}, cfg);
    mm = compute_metrics(R{a}, scn, cfg);
    fprintf('[demo seed %d] %-7s success=%d collided=%d T=%.1fs L=%.2fm d_min=%.2fm replans=%d\n', ...
        seeds(1), labels{a}, mm.success, mm.collided, mm.T, mm.L, mm.d_min, mm.n_replan);
end
plot_results(scn, R, labels, cfg);
export_demo_videos(scn, R, algos, cfg, 'S1');

%% ---------------- Phần B: chạy hàng loạt --------------------------------
nS = numel(seeds);
M  = [];
for s = 1:nS
    scn = scenario_S1(seeds(s));              % cùng hạt giống -> cùng điều kiện đầu
    for a = 1:numel(algos)
        res = sim_main(scn, algos{a}, cfg);
        M_s = compute_metrics(res, scn, cfg);
        if isempty(M), M = repmat(M_s, numel(algos), nS); end
        M(a, s) = M_s; %#ok<SAGROW>
    end
    fprintf('seed %d done (%d/%d)\n', seeds(s), s, nS);
end

%% ---------------- Bảng thống kê ----------------------------------------
names = {'T','L','L_ratio','d_min','near_miss','mean_abs_w','rms_acc', ...
         'mean_jerk','turn_total','reversals','t_comp_ms'};
fprintf('\n===== S1 - %d runs / algorithm (mean +- std over SUCCESSFUL runs) =====\n', nS);
fprintf('%-12s', 'Metric');  fprintf('%-22s', labels{:});  fprintf('\n');
fprintf('%-12s', 'collision%');  for a = 1:3, fprintf('%-22.1f', 100*mean([M(a,:).collided])); end; fprintf('\n');
fprintf('%-12s', 'success%');    for a = 1:3, fprintf('%-22.1f', 100*mean([M(a,:).success]));  end; fprintf('\n');
for i = 1:numel(names)
    fprintf('%-12s', names{i});
    for a = 1:3
        ok = [M(a,:).success] == 1;
        x  = [M(a,:).(names{i})];
        x  = x(ok);
        if isempty(x), fprintf('%-22s', 'n/a');
        else, fprintf('%-22s', sprintf('%.3f +- %.3f', mean(x), std(x))); end
    end
    fprintf('\n');
end

%% ---------------- Kiểm định Wilcoxon (DWA-KF vs đối chứng) -------------
if exist('signrank', 'file')
    fprintf('\nWilcoxon signed-rank (DWA-KF vs ...), paired, 5%% level:\n');
    for other = [1 3]
        ok = [M(2,:).success] == 1 & [M(other,:).success] == 1;
        for nm = {'T', 'd_min', 'mean_jerk'}
            xa = [M(2,ok).(nm{1})];  xb = [M(other,ok).(nm{1})];
            if numel(xa) >= 5
                p = signrank(xa, xb);
                fprintf('  %-7s vs %-7s  %-10s p = %.4f %s\n', 'DWA-KF', labels{other}, nm{1}, p, ...
                        ternary(p < 0.05, '(significant)', '(n.s.)'));
            end
        end
    end
else
    fprintf('\n(Statistics Toolbox khong co -> bo qua kiem dinh Wilcoxon)\n');
end
save('results_S1.mat', 'M', 'algos', 'labels', 'seeds', 'cfg');
fprintf('\nDa luu results_S1.mat\n');
