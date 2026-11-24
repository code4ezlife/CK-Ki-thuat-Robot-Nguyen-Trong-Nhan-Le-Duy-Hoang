function res = sim_main(scn, algo, cfg)
%SIM_MAIN  Một lần chạy mô phỏng (vòng lặp mỗi 0.1 s, đúng pseudo-code mục 3.3.5).
%   algo: 'dwa' | 'dwa_kf' | 'orca'.  Các khối còn lại dùng chung để so sánh công bằng.
%   Nhiễu cảm biến và nhiễu chuyển động người được sinh SẴN từ scn.seed nên
%   cả ba thuật toán chạy trong cùng điều kiện ngẫu nhiên (mục 3.2.3, bước 5).
map = scn.map;  humans = scn.humans;  goal = scn.goal;  nH = numel(humans);
robot  = struct('x',scn.start(1), 'y',scn.start(2), 'th',scn.th0, 'v',0, 'w',0);
tracks = struct('id',{}, 'x',{}, 'P',{}, 'unseen',{}, 'z',{});
N = round(cfg.T_max/cfg.dt);

if isfield(scn, 'seed'), rng(scn.seed + 7919); else, rng(7919); end
hum_noise  = randn(N, nH, 2);
sens_noise = randn(N, nH, 2);

% nhật ký robot (N+1 mẫu, mẫu 1 là trạng thái ban đầu)
t_log = zeros(N+1,1);  x_log = t_log;  y_log = t_log;  th_log = t_log;
v_log = t_log;  w_log = t_log;  de_log = t_log;  dh_log = inf(N+1,1);  tc_log = zeros(N,1);
% nhật ký người / Kalman (mẫu k ứng với thời điểm (k-1)*dt, cùng chỉ số với robot)
hum_true = nan(N, nH, 2);  hum_est = nan(N, nH, 2);
hum_estv = nan(N, nH, 2);  hum_meas = nan(N, nH, 2);
x_log(1) = robot.x;  y_log(1) = robot.y;  th_log(1) = robot.th;
de_log(1) = edge_dist(robot, humans, map, cfg, 0);
dh_log(1) = human_dist(robot, humans, cfg, 0);

% planner toàn cục: đường đi ban đầu
path = astar_plan(map, cfg, [robot.x robot.y], goal, zeros(0,3));
if isempty(path), path = [robot.x robot.y; goal]; end
res.path0 = path;  res.n_replan = 0;  res.replan_log = {};  res.replan_t = [];  res.replan_pos = zeros(0,2);
t_last = -Inf;  pos_hist = [0 robot.x robot.y];
collided = false;  success = false;  idx = 1;  nk = 0;

for k = 1:N
    t = (k-1)*cfg.dt;  nk = k;
    xi  = reshape(hum_noise(k,:,:),  nH, 2);
    eta = reshape(sens_noise(k,:,:), nH, 2);
    for j = 1:nH, hum_true(k,j,:) = reshape(humans(j).p, 1, 1, 2); end

    [z, ids] = sense(humans, [robot.x robot.y], cfg, t, eta);   % số đo nhiễu
    tracks   = kalman_update(tracks, z, ids, cfg);              % x_hat, P
    pred     = kalman_predict(tracks, cfg);                     % p_hat(t_i), sigma(t_i)
    for m = 1:size(z,1)
        hum_meas(k, ids(m), 1) = z(m,1);  hum_meas(k, ids(m), 2) = z(m,2);
    end
    for j = 1:numel(tracks)
        hum_est(k,  tracks(j).id, 1) = tracks(j).x(1);  hum_est(k,  tracks(j).id, 2) = tracks(j).x(2);
        hum_estv(k, tracks(j).id, 1) = tracks(j).x(3);  hum_estv(k, tracks(j).id, 2) = tracks(j).x(4);
    end

    [trig, tmp, why] = replan_trigger(path, tracks, robot, pos_hist, goal, t, t_last, cfg);
    if trig
        newp = astar_plan(map, cfg, [robot.x robot.y], goal, tmp);
        if ~isempty(newp), path = newp; end
        t_last = t;  res.n_replan = res.n_replan + 1;
        res.replan_log{end+1} = sprintf('t=%.1fs (%s)', t, why);
        res.replan_t(end+1) = t;  res.replan_pos(end+1,:) = [robot.x robot.y];
    end

    la = cfg.lookahead;                                          % ORCA: 1,5 m
    if strcmp(algo, 'dwa') || strcmp(algo, 'dwa_kf'), la = cfg.lookahead_dwa; end
    g  = lookahead_point(path, [robot.x robot.y], la);

    tc = tic;
    switch algo
        case 'dwa',    [v, w] = dwa_original(robot, g, pred, map, cfg);
        case 'dwa_kf', [v, w] = dwa_kf(robot, g, pred, map, cfg);
        case 'orca',   [v, w] = orca_step(robot, g, pred, map, cfg);
        otherwise, error('Thuat toan khong hop le: %s', algo);
    end
    tc_log(k) = toc(tc);

    robot  = step_robot(robot, v, w, cfg);
    humans = step_humans(humans, t, cfg, xi);

    idx = idx + 1;
    t_log(idx) = t + cfg.dt;  x_log(idx) = robot.x;  y_log(idx) = robot.y;
    th_log(idx) = robot.th;   v_log(idx) = robot.v;  w_log(idx) = robot.w;
    de_log(idx) = edge_dist(robot, humans, map, cfg, t + cfg.dt);
    dh_log(idx) = human_dist(robot, humans, cfg, t + cfg.dt);
    pos_hist(end+1,:) = [t + cfg.dt, robot.x, robot.y]; %#ok<AGROW>

    if de_log(idx) < 0, collided = true; break; end
    if norm([robot.x robot.y] - goal) < cfg.goal_tol, success = true; break; end
end

res.t = t_log(1:idx);   res.x = x_log(1:idx);   res.y = y_log(1:idx);
res.th = th_log(1:idx); res.v = v_log(1:idx);   res.w = w_log(1:idx);
res.d_edge = de_log(1:idx);  res.d_human = dh_log(1:idx);  res.tcomp = tc_log(1:max(idx-1,1));
res.hum_true = hum_true(1:nk,:,:);  res.hum_est  = hum_est(1:nk,:,:);
res.hum_estv = hum_estv(1:nk,:,:);  res.hum_meas = hum_meas(1:nk,:,:);
res.collided = collided;  res.success = success;  res.path_final = path;
res.algo = algo;
end

function d = edge_dist(robot, humans, map, cfg, t)
% khoảng cách MÉP-MÉP nhỏ nhất tới vật cản thật (kệ, cột, người đã xuất hiện); < 0: va chạm.
% Biên bản đồ KHÔNG tính là vật cản khi đo d_min / suýt va chạm; chỉ khi robot ĐI RA
% NGOÀI bản đồ (khoảng cách tới biên < r_robot) mới tính là va chạm.
pos = [robot.x robot.y];
d   = dist_static(pos, map, false) - cfg.r_robot;
for j = 1:numel(humans)
    if t >= humans(j).t_start
        d = min(d, norm(humans(j).p - pos) - cfg.r_human - cfg.r_robot);
    end
end
b  = map.bounds;
dw = min([pos(1)-b(1), b(2)-pos(1), pos(2)-b(3), b(4)-pos(2)]) - cfg.r_robot;
if dw < 0, d = min(d, dw); end
end

function d = human_dist(robot, humans, cfg, t)
% khoảng cách mép-mép nhỏ nhất tới NGƯỜI đã xuất hiện (Inf nếu chưa có ai)
d = Inf;
for j = 1:numel(humans)
    if t >= humans(j).t_start
        d = min(d, norm(humans(j).p - [robot.x robot.y]) - cfg.r_human - cfg.r_robot);
    end
end
end
