function [V, W, X, Y, THend] = dwa_samples(robot, cfg)
%DWA_SAMPLES  Cửa sổ động V_r = V_s ∩ V_d (mục 2.2.5), lấy mẫu lưới nv x nw,
%   và mô phỏng quỹ đạo T = 2 s bằng Euler (2.4) cho TẤT CẢ ứng viên cùng lúc.
%   X, Y : (nc x n) toạ độ các điểm quỹ đạo;  THend: góc hướng cuối quỹ đạo.
dt = cfg.dt;
vlo = max(0,           robot.v - cfg.a_v*dt);
vhi = min(cfg.v_max,   robot.v + cfg.a_v*dt);
wlo = max(-cfg.w_max,  robot.w - cfg.a_w*dt);
whi = min( cfg.w_max,  robot.w + cfg.a_w*dt);
[Vg, Wg] = ndgrid(linspace(vlo, vhi, cfg.nv), linspace(wlo, whi, cfg.nw));
V = Vg(:);  W = Wg(:);
n = cfg.n_pred;  d = cfg.delta;
TH    = robot.th + W*(d*(0:n-1));            % nc x n
X     = robot.x + cumsum(V.*cos(TH)*d, 2);
Y     = robot.y + cumsum(V.*sin(TH)*d, 2);
THend = robot.th + W*d*n;
end
