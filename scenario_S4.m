function scn = scenario_S4(seed, cfg)
%SCENARIO_S4  Kịch bản S4 - Nhiều người di chuyển (Bảng 3.1 / Hình 3.2).
%   Không gian trống 12x12 m. Robot đi (1;1) -> (11;11). cfg.s4_nhum (=6) người đi CẮT NGANG
%   bản đồ: mỗi người xuất phát cách biên 0,8 m ở một cạnh ngẫu nhiên rồi đi tới một điểm
%   ngẫu nhiên ở cạnh đối diện (điểm đích nằm ngoài bản đồ 1,5 m, để người đi ra khỏi cảnh),
%   tốc độ ưa thích U(0,8; 1,4) m/s. Người không né robot lẫn nhau.
%   Ngẫu nhiên theo hạt giống (mục 3.3.7): xuất phát robot lệch +-0,5 m, thời điểm xuất hiện
%   của từng người lệch ngẫu nhiên 0-1 s, hướng/điểm xuất phát/tốc độ của từng người.
%   Điểm xuất phát của người được chọn cách điểm đầu robot >= 3 m, cách đích robot >= 2 m,
%   và cách nhau >= 1 m (để điều kiện đầu không vô lý).
if nargin < 1, seed = 101; end
if nargin < 2, cfg = config_params(); end
rng(seed);

map.bounds  = [0 12 0 12];
map.rects   = zeros(0,4);
map.circles = zeros(0,3);
goal  = [11 11];
map.goal       = goal;
map.orca_discs = make_orca_discs(map);

start = [1 1] + (2*rand(1,2) - 1) * 0.5;

in = 0.8;  lo = in;  hi = 12 - in;              % vùng xuất phát cách biên 0,8 m
out = 1.5;                                       % đích nằm ngoài bản đồ 1,5 m
humans = struct('p',{}, 'v',{}, 'goal',{}, 'speed',{}, 't_start',{});
P0 = zeros(0,2);
n = 0;
while n < cfg.s4_nhum
    side = randi(4);                             % 1 trái, 2 phải, 3 dưới, 4 trên
    u = lo + (hi - lo)*rand;   w = lo + (hi - lo)*rand;
    switch side
        case 1, p0 = [lo u];  pg = [12+out w];
        case 2, p0 = [hi u];  pg = [-out    w];
        case 3, p0 = [u lo];  pg = [w -out];
        case 4, p0 = [u hi];  pg = [w 12+out];
    end
    spd = 0.8 + 0.6*rand;                        % 0,8 - 1,4 m/s
    ts  = rand;                                  % 0 - 1 s
    ok = norm(p0 - start) >= 3 && norm(p0 - goal) >= 2;
    if ok && ~isempty(P0), ok = all(sqrt(sum((P0 - p0).^2, 2)) >= 1); end
    if ~ok, continue; end
    n = n + 1;  P0(n,:) = p0;                    %#ok<AGROW>
    d = (pg - p0)/norm(pg - p0);
    humans(n) = struct('p',p0, 'v',spd*d, 'goal',pg, 'speed',spd, 't_start',ts); %#ok<AGROW>
end

scn.name   = 'S4';
scn.seed   = seed;
scn.map    = map;
scn.start  = start;
scn.goal   = goal;
scn.th0    = atan2(goal(2)-start(2), goal(1)-start(1));
scn.humans = humans;
end
