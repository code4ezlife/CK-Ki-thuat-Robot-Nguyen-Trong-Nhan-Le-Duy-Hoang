%% LOOKAHEAD_S2  Ảnh hưởng của khoảng nhìn trước d (pure pursuit) lên S2.
%  Với chân trời DWA T = 2 s, điểm cuối quỹ đạo trùng điểm mục tiêu cục bộ khi v = d/T.
%  d = 1,5 m  =>  DWA "tự khóa" tốc độ hành trình ~0,75 m/s (< v_max = 1 m/s).
%  d >= 2 m   =>  robot chạy v_max, DWA gốc (coi người đứng yên) mất khả năng né.
clear; clc; close all;
base  = config_params();
quick = false;                              % true: 5 hạt giống
seeds = base.seeds_eval;  if quick, seeds = seeds(1:5); end
algos = {'dwa','dwa_kf','orca'};  labels = {'DWA','DWA-KF','ORCA'};
LA    = [1.5 2.0 2.5];     % 2,0 = v_max*T là mặc định mới
fprintf('%-6s %-8s %-9s %-9s %-10s %-10s %-8s\n','d [m]','algo','collis%','T [s]','d_min','near_miss','v_cruise');
for L = LA
    cfg = base;  cfg.lookahead = L;  cfg.lookahead_dwa = L;   % áp dụng chung cho cả ba
    for a = 1:numel(algos)
        Mc = [];  vc = [];
        for s = 1:numel(seeds)
            scn = scenario_S2(seeds(s), cfg);
            res = sim_main(scn, algos{a}, cfg);
            m   = compute_metrics(res, scn, cfg);
            if isempty(Mc), Mc = repmat(m, 1, numel(seeds)); end
            Mc(s) = m;                                            %#ok<SAGROW>
            vc(end+1) = mean(res.v(res.t >= 1 & res.t <= 2));     %#ok<SAGROW>  % tốc độ khi chưa có xung đột
        end
        ok = [Mc.success] == 1;
        fprintf('%-6.1f %-8s %-9.1f %-9.1f %-10.2f %-10.2f %-8.2f\n', L, labels{a}, ...
            100*mean([Mc.collided]), mean([Mc(ok).T]), mean([Mc(ok).d_min]), mean([Mc(ok).near_miss]), mean(vc));
    end
end
