%% SENSITIVITY_S5  Độ nhạy của DWA-KF theo kappa trên S5 (người 2 rẽ đột ngột => dự đoán sai).
%  Mục đích: kiểm tra xem phần phồng bán kính theo độ bất định (kappa*sigma) có giúp DWA-KF
%  an toàn khi dự đoán Kalman bị sai hay không (kappa = 0 là KHÔNG dùng độ bất định).
%  Cấu hình giống run_S5 (fallback = 'least_violation'); sigma_z giữ 0,1 m.
clear; clc; close all;
base = config_params();
base.fallback = 'least_violation';
quick = false;                         % true: 5 hạt giống
seeds = base.seeds_eval;  if quick, seeds = seeds(1:5); end
kappas = [0 1 2 3];
nK = numel(kappas);  nS = numel(seeds);
out = zeros(nK, 8);   % collis%, success%, timeout%, d_min_h, near_miss_h, T, L_ratio, n_replan

for c = 1:nK
    cfg = base;  cfg.kappa = kappas(c);
    Mc = [];
    for s = 1:nS
        scn = scenario_S5(seeds(s), cfg);
        res = sim_main(scn, 'dwa_kf', cfg);
        m   = compute_metrics(res, scn, cfg);
        if isempty(Mc), Mc = repmat(m, 1, nS); end
        Mc(s) = m; %#ok<SAGROW>
    end
    ok = [Mc.success] == 1;
    out(c,:) = [100*mean([Mc.collided]), 100*mean([Mc.success]), 100*mean(~[Mc.collided] & ~[Mc.success]), ...
                mean([Mc.d_min_h]), mean([Mc.near_miss_h]), mean([Mc(ok).T]), mean([Mc(ok).L_ratio]), mean([Mc.n_replan])];
    Mall{c} = Mc; %#ok<SAGROW>
    fprintf('kappa=%g done\n', kappas(c));
end

fprintf('\n===== S5 sensitivity of DWA-KF to kappa (%d runs each, fallback = %s) =====\n', nS, base.fallback);
fprintf('%-7s %-9s %-10s %-10s %-12s %-12s %-8s %-9s %-9s\n', 'kappa', 'collis%', 'success%', 'timeout%', 'd_min_h [m]', 'near_miss_h', 'T [s]', 'L_ratio', 'replans');
for c = 1:nK
    fprintf('%-7g %-9.1f %-10.1f %-10.1f %-12.3f %-12.2f %-8.2f %-9.2f %-9.2f\n', kappas(c), out(c,:));
end

% McNemar chính xác: kappa = 0 so với từng kappa > 0 (va chạm, theo cặp cùng hạt giống)
fprintf('\nMcNemar exact (collision), kappa = 0 vs ...\n');
for c = 2:nK
    b = sum( [Mall{1}.collided] & ~[Mall{c}.collided]);     % chỉ kappa = 0 va chạm
    d = sum(~[Mall{1}.collided] &  [Mall{c}.collided]);     % chỉ kappa > 0 va chạm
    n = b + d;
    if n == 0, p = 1; else, k = 0:min(b, d); p = min(1, 2*sum(exp(gammaln(n+1) - gammaln(k+1) - gammaln(n-k+1) + n*log(0.5)))); end
    fprintf('  vs kappa=%g : only kappa=0 collided %d, only kappa=%g collided %d, p = %.3g\n', kappas(c), b, kappas(c), d, p);
end
save('results_S5_sensitivity.mat', 'kappas', 'out', 'seeds');

figure('Name','S5 - Sensitivity to kappa','Color','w');
subplot(1,2,1); hold on; grid on;
plot(kappas, out(:,1), 'o-', 'LineWidth',1.5);  plot(kappas, out(:,2), 's-', 'LineWidth',1.5);  plot(kappas, out(:,3), '^-', 'LineWidth',1.5);
xlabel('\kappa');  ylabel('rate [%]');  legend('collision','success','timeout','Location','best');  ylim([0 100]);
title('S5: outcomes vs \kappa');
subplot(1,2,2); hold on; grid on;
yyaxis left;  plot(kappas, out(:,4), 'o-', 'LineWidth',1.5);  ylabel('d_{min,human} [m]');  ylim([0 max(1.5, 1.1*max(out(:,4)))]);
yyaxis right; plot(kappas, out(:,6), 's-', 'LineWidth',1.5);  ylabel('T [s]');
xlabel('\kappa');  title('S5: clearance and time vs \kappa');
