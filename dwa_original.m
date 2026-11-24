function [v, w] = dwa_original(robot, g, pred, map, cfg)
%DWA_ORIGINAL  DWA thông thường (đối chứng 1, mục 3.3.4).
%   Vật cản ĐỘNG được lấy tại vị trí đo hiện tại và coi là ĐỨNG YÊN suốt quỹ đạo.
%   Điểm quỹ đạo (3.2):  J = w_h*h + w_g*g + w_c*c + w_v*s ;  d_min < 0 bị loại.
[V, W, X, Y, THend] = dwa_samples(robot, cfg);
nc = numel(V);  n = cfg.n_pred;

% vật cản tĩnh
D = reshape(dist_static([X(:) Y(:)], map), nc, n) - cfg.r_robot - cfg.m_static;
dmin = min(D, [], 2);
% vật cản động: vị trí hiện tại, đứng yên
for j = 1:numel(pred)
    Dd = sqrt((X - pred(j).z(1)).^2 + (Y - pred(j).z(2)).^2) ...
         - (pred(j).r + cfg.r_robot + cfg.m);
    dmin = min(dmin, min(Dd, [], 2));
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
J  = cfg.w_h*h + cfg.w_g*gp + cfg.w_c*c + cfg.w_v*s;
J(~ok) = -Inf;
[~, i] = max(J);
v = V(i);  w = W(i);
end
