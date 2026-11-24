function [v, w] = dwa_kf(robot, g, pred, map, cfg)
%DWA_KF  DWA có dự đoán quỹ đạo Kalman (phương pháp đề xuất, mục 3.3.5).
%   - Kiểm tra va chạm THEO THỜI GIAN với vị trí dự đoán p_hat_j(t_i) và bán kính
%     an toàn tăng theo độ bất định: r_safe = r_j + r_robot + m + kappa*sigma_j(t_i)  (3.3)
%   - Nối thêm đoạn phanh v^2/(2 a_v) vào cuối quỹ đạo.
%   - J_KF = w_h*h + w_g*g + w_c*c(d_min) + w_v*s - w_r*rho                        (3.4)
%   Khi không có vật cản động (pred rỗng, như S1) hàm chỉ khác DWA gốc ở đoạn phanh.
[V, W, X, Y, THend] = dwa_samples(robot, cfg);
nc = numel(V);  n = cfg.n_pred;  d = cfg.delta;

% đoạn phanh nối thêm: tốc độ giảm đều với gia tốc a_v, giữ nguyên w
nt  = ceil(cfg.v_max/(cfg.a_v*d));
vt  = max(V - cfg.a_v*d*(1:nt), 0);
THt = THend + W*(d*(0:nt-1));
Xt  = X(:,end) + cumsum(vt.*cos(THt)*d, 2);
Yt  = Y(:,end) + cumsum(vt.*sin(THt)*d, 2);
XA  = [X Xt];  YA = [Y Yt];  na = n + nt;

% vật cản tĩnh (bản đồ khoảng cách)
D    = reshape(dist_static([XA(:) YA(:)], map), nc, na) - cfg.r_robot - cfg.m_static;
dmin = min(D, [], 2);

% vật cản động theo thời gian
rho = zeros(nc,1);
for j = 1:numel(pred)
    idx = min(1:na, n);                          % đoạn phanh: dùng điểm dự đoán cuối
    Pj  = pred(j).p(idx,:);
    rs  = pred(j).r + cfg.r_robot + cfg.m + cfg.kappa*pred(j).sigma(idx);   % na x 1
    Dd  = sqrt((XA - Pj(:,1)').^2 + (YA - Pj(:,2)').^2) - rs';              % nc x na
    dmin = min(dmin, min(Dd, [], 2));
    [hit, tw] = max(Dd(:,1:n) < cfg.d_warn, [], 2);      % thời điểm cảnh báo đầu tiên
    rho = max(rho, hit .* (1 - (tw*d)/cfg.T_pred));
end
ok = dmin >= 0;
if ~any(ok)
    if strcmp(cfg.fallback, 'least_violation')       % né tiếp theo hướng ít vi phạm nhất
        [~, i] = max(dmin);  v = V(i);  w = W(i);
    else
        [v, w] = emergency_brake(robot, map, cfg);   % đúng mô tả báo cáo mục 3.3.4
    end
    return
end

p0 = [robot.x robot.y];
Xe = X(:,end);  Ye = Y(:,end);
h  = 1 - abs(wrapPi(atan2(g(2)-Ye, g(1)-Xe) - THend))/pi;
gp = 1 - min(sqrt((Xe-g(1)).^2 + (Ye-g(2)).^2) / max(norm(p0-g), 1e-6), 1);
c  = min(dmin, cfg.d_c) / cfg.d_c;
s  = V / cfg.v_max;
J  = cfg.w_h*h + cfg.w_g*gp + cfg.w_c*c + cfg.w_v*s - cfg.w_r*rho;
J(~ok) = -Inf;
[~, i] = max(J);
v = V(i);  w = W(i);
end
