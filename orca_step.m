function [v, w] = orca_step(robot, g, pred, map, cfg)
%ORCA_STEP  ORCA thích nghi cho robot vi sai (đối chứng 2, mục 2.2.7 / 3.3.6).
%   - Robot nhận TOÀN BỘ trách nhiệm tránh (dùng u thay vì u/2) vì người không hợp tác.
%   - Vận tốc người lấy từ ước lượng Kalman (pred.vel).
%   - Vật cản tĩnh xấp xỉ bằng các đĩa tròn đứng yên (map.orca_discs); tường biên
%     được thêm như 4 nửa mặt phẳng.
%   - Vận tốc 2D kết quả đổi sang (v, w):
%        w = K*(theta_d - theta),  v = |v_new| * max(0, cos(theta_d - theta))
p  = [robot.x robot.y];
vA = robot.v*[cos(robot.th) sin(robot.th)];
tau = cfg.orca_tau;  rA = cfg.r_robot;  mg = cfg.orca_margin;

% vận tốc ưa thích: hướng tới mục tiêu cục bộ, giảm dần khi gần đích
dg = norm(map.goal - p);  dr = g - p;
if norm(dr) < 1e-9
    vpref = [0 0];
else
    vpref = cfg.v_max*min(1, dg/1.5)*dr/norm(dr);
end

L = zeros(0,3);
% -- vật cản tĩnh (đĩa): chỉ lấy cfg.orca_K đĩa gần nhất trong bán kính láng giềng
D  = map.orca_discs;
dd = sqrt((D(:,1)-p(1)).^2 + (D(:,2)-p(2)).^2) - D(:,3);
cand = find(dd < cfg.orca_neigh);
[~, ord] = sort(dd(cand));
cand = cand(ord(1:min(end, cfg.orca_K)));
for q = 1:numel(cand)
    L(end+1,:) = orca_line(p, vA, [0 0], D(cand(q),1:2), rA + D(cand(q),3) + mg, tau, cfg.dt); %#ok<AGROW>
end
% -- vật cản động (từ Kalman)
for j = 1:numel(pred)
    if norm(pred(j).pos - p) - pred(j).r > cfg.orca_neigh, continue; end
    L(end+1,:) = orca_line(p, vA, pred(j).vel, pred(j).pos, rA + pred(j).r + cfg.orca_margin_dyn, tau, cfg.dt); %#ok<AGROW>
end
% -- tường biên: n·v >= -dw/tau
b = map.bounds;
walls = [1 0 b(1); -1 0 -b(2); 0 1 b(3); 0 -1 -b(4)];
for i = 1:4
    dw = walls(i,1:2)*p' - walls(i,3) - rA - mg;
    if dw < cfg.orca_neigh
        L(end+1,:) = [walls(i,1:2), -dw/tau]; %#ok<AGROW>
    end
end

vn = orca_lp(L, vpref, cfg.v_max);

% đổi sang lệnh robot vi sai
if norm(vn) < 1e-6
    thd = robot.th;
else
    thd = atan2(vn(2), vn(1));
end
e = wrapPi(thd - robot.th);
w = max(-cfg.w_max, min(cfg.w_max, cfg.orca_Kw*e));
v = norm(vn)*max(0, cos(e));
end

% ------------------------------------------------------------------------
function row = orca_line(pA, vA, vB, pB, r, tau, dt)
% Ràng buộc ORCA(A|B) dạng [nx ny c]:  n·v >= c, robot nhận toàn bộ u.
rp = pB - pA;  rv = vA - vB;
d2 = rp*rp';   r2 = r*r;
if d2 > r2                                   % chưa chạm
    w  = rv - rp/tau;  w2 = w*w';  dot1 = w*rp';
    if dot1 < 0 && dot1^2 > r2*w2            % chiếu lên mặt cắt (cutoff circle)
        wl = sqrt(w2);  uw = w/wl;
        n = uw;  u = (r/tau - wl)*uw;
    else                                     % chiếu lên cạnh bên của hình nón
        leg = sqrt(d2 - r2);
        if rp(1)*w(2) - rp(2)*w(1) > 0       % cạnh trái
            dr = [rp(1)*leg - rp(2)*r,  rp(1)*r + rp(2)*leg]/d2;
        else                                 % cạnh phải
            dr = -[rp(1)*leg + rp(2)*r, -rp(1)*r + rp(2)*leg]/d2;
        end
        n = [-dr(2) dr(1)];
        u = (rv*dr')*dr - rv;
    end
else                                         % đã chồng lấn biên an toàn
    w  = rv - rp/dt;  wl = norm(w);
    if wl < 1e-9, uw = -rp/max(norm(rp), 1e-9); else, uw = w/wl; end
    n = uw;  u = (r/dt - wl)*uw;
end
row = [n, n*(vA + u)'];
end
