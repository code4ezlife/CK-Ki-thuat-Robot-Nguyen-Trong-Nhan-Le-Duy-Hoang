%% SENSITIVITY_S2  Khảo sát độ nhạy của DWA-KF trên S2 (mục 3.3.8):
%   - hệ số phồng kappa = 0, 1, 2, 3     (sigma_z = 0,1 m)
%   - mức nhiễu cảm biến sigma_z = 0,05; 0,1; 0,2 m   (kappa = 2)
%   Bộ lọc Kalman luôn dùng đúng sigma_z của cảm biến (R = sigma_z^2 I).
clear; clc; close all;
base  = config_params();
quick = false;                     % true: 5 hạt giống
seeds = base.seeds_eval;  if quick, seeds = seeds(1:5); end
cases = [ 0 0.10;  1 0.10;  2 0.10;  3 0.10;     % [kappa sigma_z]
          2 0.05;  2 0.20 ];
nC = size(cases, 1);  nS = numel(seeds);
out = zeros(nC, 7);   % collision%, success%, d_min mean, d_min std, near_miss mean, T mean, jerk mean

for c = 1:nC
    cfg = base;  cfg.kappa = cases(c,1);  cfg.sigma_z = cases(c,2);
    Mc = [];
    for s = 1:nS
        scn = scenario_S2(seeds(s), cfg);
        res = sim_main(scn, 'dwa_kf', cfg);
        m   = compute_metrics(res, scn, cfg);
        if isempty(Mc), Mc = repmat(m, 1, nS); end
        Mc(s) = m; %#ok<SAGROW>
    end
    ok = [Mc.success] == 1;
    out(c,:) = [100*mean([Mc.collided]), 100*mean([Mc.success]), ...
                mean([Mc(ok).d_min]), std([Mc(ok).d_min]), mean([Mc(ok).near_miss]), ...
                mean([Mc(ok).T]), mean([Mc(ok).mean_jerk])];
    fprintf('kappa=%g sigma_z=%.2f done\n', cases(c,1), cases(c,2));
end

fprintf('\n===== S2 sensitivity of DWA-KF (%d runs each) =====\n', nS);
fprintf('%-7s %-9s %-10s %-10s %-16s %-10s %-8s %-8s\n', ...
        'kappa', 'sigma_z', 'collis%', 'success%', 'd_min [m]', 'near_miss', 'T [s]', 'jerk');
for c = 1:nC
    fprintf('%-7g %-9.2f %-10.1f %-10.1f %-16s %-10.2f %-8.2f %-8.2f\n', cases(c,1), cases(c,2), ...
        out(c,1), out(c,2), sprintf('%.3f +- %.3f', out(c,3), out(c,4)), out(c,5), out(c,6), out(c,7));
end
save('results_S2_sensitivity.mat', 'cases', 'out', 'seeds');

% biểu đồ
figure('Name','S2 - Sensitivity','Color','w');
subplot(1,2,1); idx = 1:4;
yyaxis left;  plot(cases(idx,1), out(idx,3), 'o-', 'LineWidth',1.5);  ylabel('d_{min} [m]');
yyaxis right; plot(cases(idx,1), out(idx,5), 's-', 'LineWidth',1.5);  ylabel('near-miss events');
xlabel('\kappa');  title('Effect of \kappa (\sigma_z = 0.1 m)');  grid on;
subplot(1,2,2); idx = [5 3 6];
yyaxis left;  plot(cases(idx,2), out(idx,3), 'o-', 'LineWidth',1.5);  ylabel('d_{min} [m]');
yyaxis right; plot(cases(idx,2), out(idx,5), 's-', 'LineWidth',1.5);  ylabel('near-miss events');
xlabel('\sigma_z [m]');  title('Effect of sensor noise (\kappa = 2)');  grid on;
