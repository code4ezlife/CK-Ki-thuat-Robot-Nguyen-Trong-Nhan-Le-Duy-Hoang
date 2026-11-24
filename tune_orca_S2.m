%% TUNE_ORCA_S2  Hiệu chỉnh biên an toàn vật cản ĐỘNG của ORCA trên tập hạt giống 1-20
%  (mục 3.2.3, bước 4). Chọn giá trị có tỉ lệ va chạm thấp nhất, rồi thời gian ngắn nhất.
%  Giá trị chọn được gán vào cfg.orca_margin_dyn trong config_params.m.
clear; clc;
base = config_params();  seeds = base.seeds_tune;
margins = [0.12 0.3 0.5 0.7 0.9];
fprintf('%-10s %-10s %-10s %-8s %-8s %-8s\n', 'margin', 'collis', 'success', 'T [s]', 'd_min', 'near');
for md = margins
    cfg = base;  cfg.orca_margin_dyn = md;
    C = 0;  S = 0;  T = [];  D = [];  NM = [];
    for sd = seeds
        scn = scenario_S2(sd, cfg);
        m = compute_metrics(sim_main(scn, 'orca', cfg), scn, cfg);
        C = C + m.collided;  S = S + m.success;
        if m.success, T(end+1) = m.T; D(end+1) = m.d_min; NM(end+1) = m.near_miss; end %#ok<SAGROW>
    end
    fprintf('%-10.2f %-10s %-10s %-8.1f %-8.2f %-8.2f\n', md, sprintf('%d/%d',C,numel(seeds)), ...
            sprintf('%d/%d',S,numel(seeds)), mean(T), mean(D), mean(NM));
end
