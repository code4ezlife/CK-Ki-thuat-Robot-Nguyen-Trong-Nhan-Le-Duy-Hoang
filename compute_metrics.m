function m = compute_metrics(res, scn, cfg)
%COMPUTE_METRICS  Các chỉ tiêu ở Bảng 2.2 cho MỘT lần chạy.
dt = cfg.dt;
m.collided = res.collided;
m.success  = res.success;
m.T        = res.t(end);                                         % thời gian [s]
m.L        = sum(sqrt(diff(res.x).^2 + diff(res.y).^2));          % quãng đường [m]
m.L_ratio  = m.L / norm(scn.goal - scn.start);                    % L / L_thẳng
m.d_min    = min(res.d_edge);                                     % khoảng cách mép nhỏ nhất [m]
nm         = res.d_edge(:) < cfg.near_miss;
m.near_miss = sum(diff([0; nm]) == 1);                            % số sự kiện suýt va chạm
m.d_min_h  = min(res.d_human);                                    % tới NGƯỜI (Inf nếu không có người)
nh         = res.d_human(:) < cfg.near_miss;
m.near_miss_h = sum(diff([0; nh]) == 1);
a          = diff(res.v)/dt;                                      % gia tốc dài
jk         = diff(a)/dt;                                          % độ giật
m.mean_abs_w = mean(abs(res.w));
m.rms_acc    = sqrt(mean(a.^2));
m.mean_jerk  = mean(abs(jk));
m.turn_total = sum(abs(diff(res.th)));                            % tổng góc quay [rad]
ws = res.w(abs(res.w) > 0.05);
m.reversals  = sum(diff(sign(ws)) ~= 0);                          % số lần đảo chiều quay
m.t_comp_ms  = 1000*mean(res.tcomp);                              % thời gian tính / chu kỳ [ms]
m.n_replan   = res.n_replan;
if isempty(res.replan_t), m.t_replan1 = NaN; else, m.t_replan1 = res.replan_t(1); end
end
