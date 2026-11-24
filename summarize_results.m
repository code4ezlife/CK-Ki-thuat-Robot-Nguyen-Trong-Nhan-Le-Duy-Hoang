function summarize_results(M, labels, ttl)
%SUMMARIZE_RESULTS  Bảng thống kê và kiểm định thống kê theo cặp (DWA-KF = hàng 2 vs từng đối chứng).
%   - Cột mean +- std của các chỉ tiêu chỉ tính trên lượt THÀNH CÔNG; dòng n_success cho biết cỡ mẫu.
%     (Thuật toán va chạm nhiều thì cỡ mẫu nhỏ và thiên lệch -> đọc cùng collision%.)
%   - Va chạm và thành công: kiểm định McNemar chính xác (theo cặp, cùng hạt giống).
%   - d_min, near_miss: Wilcoxon trên TẤT CẢ các cặp (tính cả lượt va chạm).
%   - T, mean_jerk: Wilcoxon chỉ trên các cặp mà cả hai đều thành công (ghi rõ n).
[nA, nS] = size(M);
names = {'T','L','L_ratio','d_min','near_miss','d_min_h','near_miss_h','mean_abs_w','rms_acc', ...
         'mean_jerk','turn_total','reversals','n_replan','t_replan1','t_comp_ms'};
fprintf('\n===== %s - %d runs / algorithm (mean +- std over SUCCESSFUL runs) =====\n', ttl, nS);
fprintf('%-12s', 'Metric');  fprintf('%-22s', labels{:});  fprintf('\n');
fprintf('%-12s', 'collision%');  for a = 1:nA, fprintf('%-22.1f', 100*mean([M(a,:).collided])); end; fprintf('\n');
fprintf('%-12s', 'success%');    for a = 1:nA, fprintf('%-22.1f', 100*mean([M(a,:).success]));  end; fprintf('\n');
fprintf('%-12s', 'timeout%');    for a = 1:nA, fprintf('%-22.1f', 100*mean(~[M(a,:).collided] & ~[M(a,:).success])); end; fprintf('\n');
fprintf('%-12s', 'n_success');   for a = 1:nA, fprintf('%-22d', sum([M(a,:).success]));         end; fprintf('\n');
for i = 1:numel(names)
    fprintf('%-12s', names{i});
    for a = 1:nA
        ok = [M(a,:).success] == 1;
        x  = [M(a,:).(names{i})];
        x  = x(ok);  x = x(isfinite(x));
        if isempty(x), fprintf('%-22s', 'n/a');
        else, fprintf('%-22s', sprintf('%.3f +- %.3f', mean(x), std(x))); end
    end
    fprintf('\n');
end

% ---------------- kiểm định theo cặp: DWA-KF vs từng đối chứng ----------
fprintf('\nPaired tests, DWA-KF vs ... (5%% level)\n');
for other = [1 3]
    kf = M(2,:);  ot = M(other,:);
    % 1) va chạm: McNemar chính xác
    b = sum(~[kf.collided] &  [ot.collided]);      % chỉ đối chứng va chạm
    c = sum( [kf.collided] & ~[ot.collided]);      % chỉ DWA-KF va chạm
    fprintf('  vs %-7s collision  (only %s collided: %d, only DWA-KF collided: %d)  McNemar exact p = %.2g %s\n', ...
            labels{other}, labels{other}, b, c, mcnemar_exact(b, c), sig(mcnemar_exact(b, c)));
    % 1b) thành công (đến đích mà không va chạm): McNemar chính xác
    b = sum( [ot.success] & ~[kf.success]);        % chỉ đối chứng thành công
    c = sum(~[ot.success] &  [kf.success]);        % chỉ DWA-KF thành công
    fprintf('  vs %-7s success    (only %s succeeded: %d, only DWA-KF succeeded: %d)  McNemar exact p = %.2g %s\n', ...
            labels{other}, labels{other}, b, c, mcnemar_exact(b, c), sig(mcnemar_exact(b, c)));
    if ~exist('signrank', 'file'), continue; end
    % 2) d_min, near_miss: tất cả các cặp
    for nm = {'d_min', 'near_miss', 'd_min_h', 'near_miss_h'}
        xa = [kf.(nm{1})];  xb = [ot.(nm{1})];
        if all(isfinite(xa)) && all(isfinite(xb)) && any(xa ~= xb)
            p = signrank(xa, xb);
            fprintf('  vs %-7s %-10s all %d pairs        Wilcoxon p = %.4f %s\n', labels{other}, nm{1}, nS, p, sig(p));
        end
    end
    % 3) T, mean_jerk: chỉ cặp cùng thành công
    ok = [kf.success] == 1 & [ot.success] == 1;
    for nm = {'T', 'mean_jerk'}
        xa = [kf(ok).(nm{1})];  xb = [ot(ok).(nm{1})];
        if numel(xa) >= 5 && any(xa ~= xb)
            p = signrank(xa, xb);
            if numel(xa) < 10, warn = '  [n nho, than trong]'; else, warn = ''; end
            fprintf('  vs %-7s %-10s %d both-success pairs Wilcoxon p = %.4f %s%s\n', ...
                    labels{other}, nm{1}, numel(xa), p, sig(p), warn);
        end
    end
end
if ~exist('signrank', 'file')
    fprintf('  (Statistics Toolbox khong co -> bo qua Wilcoxon; McNemar o tren khong can toolbox)\n');
end
end

function p = mcnemar_exact(b, c)
% kiểm định McNemar chính xác (hai phía) = kiểm định nhị thức cho b trên b+c với xác suất 0,5
n = b + c;
if n == 0, p = 1; return; end
k  = 0:min(b, c);
lp = gammaln(n+1) - gammaln(k+1) - gammaln(n-k+1) + n*log(0.5);
p  = min(1, 2*sum(exp(lp)));
end

function s = sig(p)
if p < 0.05, s = '(significant)'; else, s = '(n.s.)'; end
end
