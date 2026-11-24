function scn = scenario_S2(seed, cfg)
%SCENARIO_S2  Kịch bản S2 - Một người cắt ngang (Bảng 3.1 / Hình 3.2).
%   Không gian trống 12x12 m. Robot đi (1;6) -> (11;6). Một người đi (6;0,8) -> (6;11)
%   với tốc độ ưa thích 1,2 m/s và TỚI GIAO ĐIỂM CÙNG LÚC với robot.
%   Ngẫu nhiên theo hạt giống (mục 3.3.7):
%     - vị trí xuất phát robot lệch +-0,5 m
%     - tốc độ ưa thích của người 1,2 +- 0,2 m/s
%     - thời điểm người xuất hiện lệch ngẫu nhiên so với mốc "cùng tới giao điểm"
%       (khoảng lệch cfg.s2_jitter, mặc định [-0.5 0.5] s)
if nargin < 1, seed = 101; end
if nargin < 2, cfg = config_params(); end
rng(seed);

map.bounds  = [0 12 0 12];
map.rects   = zeros(0,4);              % không có vật cản tĩnh
map.circles = zeros(0,3);
goal = [11 6];
map.goal       = goal;
map.orca_discs = make_orca_discs(map);

start = [1 6] + (2*rand(1,2) - 1) * 0.5;
speed = 1.2 + (2*rand - 1) * 0.2;
jit   = cfg.s2_jitter(1) + rand * (cfg.s2_jitter(2) - cfg.s2_jitter(1));

% --- đồng bộ thời điểm tới giao điểm (x = 6) -----------------------------
h0 = [6 0.8];   hg = [6 11];
yc = start(2) + (6 - start(1))/(goal(1) - start(1))*(goal(2) - start(2));  % y robot tại x=6
Dr = hypot(6 - start(1), yc - start(2));          % quãng đường robot tới giao điểm
if Dr >= cfg.v_max^2/(2*cfg.a_v)                  % tăng tốc rồi chạy đều v_max
    t_robot = cfg.v_max/cfg.a_v + (Dr - cfg.v_max^2/(2*cfg.a_v))/cfg.v_max;
else
    t_robot = sqrt(2*Dr/cfg.a_v);
end
t_human = (yc - h0(2))/speed;                     % thời gian người tới giao điểm
t_start = max(0, t_robot - t_human + jit);        % thời điểm người xuất hiện

scn.name   = 'S2';
scn.seed   = seed;
scn.map    = map;
scn.start  = start;
scn.goal   = goal;
scn.th0    = atan2(goal(2)-start(2), goal(1)-start(1));
scn.humans = struct('p',h0, 'v',[0 speed], 'goal',hg, 'speed',speed, 't_start',t_start);
scn.info   = struct('t_robot_nominal',t_robot, 't_human_nominal',t_human, ...
                    'jitter',jit, 'cross_y',yc);
end
