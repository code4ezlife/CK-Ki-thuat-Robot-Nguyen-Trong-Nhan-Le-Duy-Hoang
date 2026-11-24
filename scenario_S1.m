function scn = scenario_S1(seed)
%SCENARIO_S1  Kịch bản S1 - Vật cản tĩnh (kho hàng), Bảng 3.1 / Hình 3.2.
%   5 kệ + 3 cột, KHÔNG có người. Robot đi (1;1) -> (11;11) trong vùng 12x12 m.
%   Ngẫu nhiên theo hạt giống: vị trí xuất phát lệch +-0.5 m.
if nargin < 1, seed = 101; end
rng(seed);

map.bounds  = [0 12 0 12];                 % [xmin xmax ymin ymax]
% Kệ hàng (chữ nhật): [xmin ymin xmax ymax]  - đọc theo Hình 3.2
map.rects   = [ 3.0  8.0  4.0 11.0;        % kệ 1
                3.0  2.5  4.0  6.5;        % kệ 2
                6.0  7.0  7.0 11.0;        % kệ 3
                6.0  0.8  7.0  5.3;        % kệ 4
                8.0  3.0  9.0  8.0];       % kệ 5
% Cột tròn: [x y r]
map.circles = [ 2.0 10.0 0.5;
                8.2  9.5 0.5;
               10.2  1.5 0.5];

goal  = [11 11];
map.goal       = goal;
map.orca_discs = make_orca_discs(map);

start = [1 1] + (2*rand(1,2) - 1) * 0.5;   % lệch +-0.5 m

scn.name   = 'S1';
scn.seed   = seed;
scn.map    = map;
scn.start  = start;
scn.goal   = goal;
scn.th0    = atan2(goal(2)-start(2), goal(1)-start(1));
% S1 không có vật cản động -> mảng rỗng (đúng cấu trúc để dùng lại cho S2..S5)
scn.humans = struct('p',{},'v',{},'goal',{},'speed',{},'t_start',{});
end
