function scn = scenario_S3(seed, cfg)
%SCENARIO_S3  Kịch bản S3 - Ngược chiều trong hành lang hẹp (Bảng 3.1 / Hình 3.2).
%   Hành lang rộng 2 m (y = 5..7), dài 12 m; hai bên là tường. Robot đi (1;6) -> (11;6).
%   Một người đi ngược chiều từ (10;6) về (1;6) với ~1,0 m/s, KHÔNG nhường đường.
%   Kiểm tra: không gian né hạn chế, "bế tắc đối xứng", dao động khi hai bên cùng né.
%   Ngẫu nhiên theo hạt giống (mục 3.3.7):
%     - vị trí xuất phát robot lệch +-0,5 m
%     - tốc độ ưa thích của người 1,0 +- 0,2 m/s
%     - thời điểm người xuất hiện lệch ngẫu nhiên 0-1 s
if nargin < 1, seed = 101; end
if nargin < 2, cfg = config_params(); end
rng(seed);

map.bounds  = [0 12 0 12];
hw = cfg.s3_width/2;                       % nửa bề rộng hành lang (mặc định 1 m)
map.rects   = [0 0 12 6-hw;                % tường dưới
               0 6+hw 12 12];              % tường trên
map.circles = zeros(0,3);
goal = [11 6];
map.goal       = goal;
map.orca_discs = make_orca_discs(map);

start   = [1 6] + (2*rand(1,2) - 1) * 0.5;
speed   = 1.0 + (2*rand - 1) * 0.2;
t_start = rand;                            % 0..1 s

scn.name   = 'S3';
scn.seed   = seed;
scn.map    = map;
scn.start  = start;
scn.goal   = goal;
scn.th0    = atan2(goal(2)-start(2), goal(1)-start(1));
scn.humans = struct('p',[10 6], 'v',[-speed 0], 'goal',[1 6], 'speed',speed, 't_start',t_start);
scn.info   = struct('speed',speed, 't_start',t_start);
end
