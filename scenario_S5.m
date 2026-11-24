function scn = scenario_S5(seed, cfg)
%SCENARIO_S5  Kịch bản S5 - Đường bị chặn và đổi hướng đột ngột (Bảng 3.1 / Hình 3.2).
%   Tường dọc tại x = 6 có CỬA rộng 1,6 m (y = 5,2..6,8) và một LỐI VÒNG phía trên
%   (y = 10,5..11,8, rộng 1,3 m, đường vòng dài hơn). Robot đi (1;6) -> (11;6).
%   Người 1: đi từ (10,5;6) về phía cửa rồi DỪNG HẲN ngay tại cửa (6,6;6) => chặn cửa.
%   Người 2: đi thẳng xuống từ (3; y0) rồi RẼ 90° VÀO ĐƯỜNG ROBOT ở giây thứ 6: đổi ngay sang
%            hướng đông, đi dọc đường đi ban đầu của robot (y ~ 6) tới x = 4,3, rồi rẽ nam và RỜI
%            KHỎI VÙNG robot (không đứng yên chặn đường). Dự đoán vận tốc không đổi của Kalman
%            bị sai ngay sau thời điểm rẽ. (Người 2 không đi qua cửa: cửa đã bị người 1 chặn.)
%   Mục đích: kiểm tra tính lại đường (T1-T3) và độ bền khi dự đoán sai.
%   Ngẫu nhiên theo hạt giống (mục 3.3.7): xuất phát robot lệch +-0,5 m; thời điểm xuất hiện
%   của từng người lệch 0-1 s; tốc độ ưa thích 1,0 +- 0,2 (người 1), 0,8 +- 0,2 (người 2);
%   y của đường rẽ lệch +-0,2 m quanh đường đi của robot.
%   cfg.s5_human2 = false: bỏ người 2 (đối chứng để tách ảnh hưởng của dự đoán sai).
if nargin < 1, seed = 101; end
if nargin < 2, cfg = config_params(); end
rng(seed);

wx = [5.85 6.15];                                  % tường dày 0,3 m
map.bounds  = [0 12 0 12];
map.rects   = [wx(1)  0.0 wx(2)  5.2;              % tường dưới
               wx(1)  6.8 wx(2) 10.5;              % tường giữa (cửa 1,6 m: y = 5,2..6,8)
               wx(1) 11.8 wx(2) 12.0];             % mẩu tường trên (lối vòng: y = 10,5..11,8)
map.circles = zeros(0,3);
goal = [11 6];
map.goal       = goal;
map.orca_discs = make_orca_discs(map);

start = [1 6] + (2*rand(1,2) - 1) * 0.5;

% --- người 1: dừng hẳn ở cửa ----------------------------------------------
sp1 = 1.0 + (2*rand - 1) * 0.2;   ts1 = rand;
h1 = struct('p',[10.5 6], 'v',[-sp1 0], 'goal',[6.6 6], 'speed',sp1, 't_start',ts1, ...
            'turn_t',Inf, 'turn_goal',[0 0], 'turned',false, 'exit_goal',[0 0], 'exited',true);

% --- người 2: đi thẳng xuống, rẽ 90° vào đường robot ở giây thứ 6, rồi rời đi ------
sp2 = 0.8 + (2*rand - 1) * 0.2;   ts2 = rand;   t_turn = 6;
yline  = start(2) + (6 - start(2))*(3 - start(1))/(goal(1) - start(1));   % y đường đi ban đầu của robot tại x = 3
y_turn = yline + (2*rand - 1) * 0.2;
y0 = y_turn + sp2*(t_turn - ts2);                  % xuất phát sao cho tới y_turn đúng lúc t = 6 s
if y0 > 11.8, sp2 = (11.8 - y_turn)/(t_turn - ts2);  y0 = 11.8; end
h2 = struct('p',[3 y0], 'v',[0 -sp2], 'goal',[3 -1.5], 'speed',sp2, 't_start',ts2, ...
            'turn_t',t_turn, 'turn_goal',[4.3 NaN], 'turned',false, 'exit_goal',[4.3 -1.5], 'exited',false);

scn.name   = 'S5';
scn.seed   = seed;
scn.map    = map;
scn.start  = start;
scn.goal   = goal;
scn.th0    = atan2(goal(2)-start(2), goal(1)-start(1));
if cfg.s5_human2, scn.humans = [h1, h2]; else, scn.humans = h1; end
scn.info   = struct('t_turn',t_turn, 'y_turn',y_turn, 'speed1',sp1, 'speed2',sp2);
end
