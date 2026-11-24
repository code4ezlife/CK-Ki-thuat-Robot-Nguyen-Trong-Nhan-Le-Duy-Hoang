function pred = kalman_predict(tracks, cfg)
%KALMAN_PREDICT  Dự đoán quỹ đạo n bước (2.11) và độ bất định sigma(t_i).
%   pred(j).p     : n x 2  vị trí dự đoán tại t_i = i*delta
%   pred(j).sigma : n x 1  căn bậc hai trị riêng lớn nhất của khối vị trí P
%   pred(j).pos / .vel / .z / .r : vị trí & vận tốc ước lượng, số đo gần nhất, bán kính
n  = cfg.n_pred;  dt = cfg.delta;
F  = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
q  = cfg.sigma_a^2;
Q1 = q*[dt^4/4 dt^3/2; dt^3/2 dt^2];
Q  = zeros(4);  Q([1 3],[1 3]) = Q1;  Q([2 4],[2 4]) = Q1;

pred = struct('id',{},'p',{},'sigma',{},'pos',{},'vel',{},'z',{},'r',{});
for j = 1:numel(tracks)
    x = tracks(j).x;  P = tracks(j).P;
    p = zeros(n,2);   sg = zeros(n,1);
    for i = 1:n
        x = F*x;  P = F*P*F' + Q;
        p(i,:) = x(1:2)';
        sg(i)  = sqrt(max(eig(P(1:2,1:2))));
    end
    pred(j) = struct('id',tracks(j).id, 'p',p, 'sigma',sg, ...
                     'pos',tracks(j).x(1:2)', 'vel',tracks(j).x(3:4)', ...
                     'z',tracks(j).z, 'r',cfg.r_human);
end
end
