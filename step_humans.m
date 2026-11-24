function humans = step_humans(humans, t, cfg, xi)
%STEP_HUMANS  Chuyển động người đi bộ theo (3.1):
%   v(k+1) = v + lambda*(v_ref - v)*dt + xi,  xi ~ N(0,(sigma_h*dt)^2 I)
%   xi: nH x 2 số ngẫu nhiên chuẩn N(0,1) cho bước này (do sim_main sinh sẵn để
%   chuyển động người GIỐNG HỆT nhau giữa các thuật toán). Người không né robot.
%   Người dừng lại khi tới cách đích của mình < 0,15 m.
%   Hành vi đặc biệt (S5): nếu có trường turn_t, tại t >= turn_t người đổi ngay sang hướng turn_goal
%   (tọa độ NaN = giữ nguyên); khi tới turn_goal, nếu có exit_goal thì rẽ tiếp để rời đi.
if nargin < 4, xi = randn(numel(humans), 2); end
for j = 1:numel(humans)
    h = humans(j);
    if t < h.t_start, continue; end
    if isfield(h, 'turn_t') && ~h.turned && t >= h.turn_t       % rẽ đột ngột (S5)
        tg = h.turn_goal;  nn = isnan(tg);  tg(nn) = h.p(nn);   % NaN = giữ nguyên tọa độ hiện tại
        h.goal = tg;  d = h.goal - h.p;
        h.v = h.speed*d/max(norm(d), 1e-9);                      % đổi hướng tức thì, giữ tốc độ
        h.turned = true;
    elseif isfield(h, 'exit_goal') && h.turned && ~h.exited && norm(h.goal - h.p) < 0.2
        h.goal = h.exit_goal;  d = h.goal - h.p;                 % tới điểm rẽ: đi rời khỏi vùng
        h.v = h.speed*d/max(norm(d), 1e-9);
        h.exited = true;
    end
    d = h.goal - h.p;  dn = norm(d);
    if dn > 0.15, vref = h.speed*d/dn; else, vref = [0 0]; end
    h.v = h.v + cfg.lambda*(vref - h.v)*cfg.dt + cfg.sigma_h*cfg.dt*xi(j,:);
    h.p = h.p + h.v*cfg.dt;
    humans(j) = h;
end
end
