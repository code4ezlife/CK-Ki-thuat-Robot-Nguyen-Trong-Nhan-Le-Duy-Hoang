function [v, w] = emergency_brake(robot, map, cfg)
%EMERGENCY_BRAKE  Không còn ứng viên an toàn: phanh (v=0) và quay tại chỗ về
%   phía có khoảng trống lớn nhất (mục 3.3.4).
v = 0;
ang  = robot.th + linspace(-pi, pi, 17);  ang(end) = [];
best = -Inf;  bi = 1;
for i = 1:numel(ang)
    c = dist_static([robot.x robot.y] + [cos(ang(i)) sin(ang(i))], map);
    if c > best, best = c; bi = i; end
end
w = max(-cfg.w_max, min(cfg.w_max, 2*wrapPi(ang(bi) - robot.th)));
end
