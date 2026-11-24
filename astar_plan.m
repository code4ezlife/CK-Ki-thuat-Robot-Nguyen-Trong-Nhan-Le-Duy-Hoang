function path = astar_plan(map, cfg, start, goal, tmp_obs)
%ASTAR_PLAN  Planner toàn cục A* trên lưới 0.1 m, 8 láng giềng, heuristic octile.
%   - Vật cản tĩnh được làm phồng thêm r_robot + 0.1 m.
%   - tmp_obs (k x 3: [x y r]) là vật cản động tạm thời khi tính lại đường.
%   - Đường tìm được được làm trơn bằng kiểm tra tầm nhìn thẳng.
%   Trả về path (K x 2) hoặc [] nếu không có đường.
if nargin < 5, tmp_obs = zeros(0,3); end
[blocked, nx, ny] = build_grid(map, cfg, tmp_obs);
b0 = map.bounds;  cs = cfg.cell;

s = to_cell(start, b0, cs, nx, ny);   g = to_cell(goal, b0, cs, nx, ny);
s = nearest_free(blocked, s);         g = nearest_free(blocked, g);

nbr  = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];   % [dy dx]
cost = [sqrt(2) 1 sqrt(2) 1 1 sqrt(2) 1 sqrt(2)];
G = inf(ny,nx);  openF = inf(ny,nx);  parent = zeros(ny,nx);
closed = false(ny,nx);
G(s(1),s(2)) = 0;
openF(s(1),s(2)) = octile(s, g);
gIdx = sub2ind([ny nx], g(1), g(2));
found = false;
while true
    [fm, cur] = min(openF(:));
    if isinf(fm), break; end
    if cur == gIdx, found = true; break; end
    openF(cur) = Inf;  closed(cur) = true;
    [cy, cx] = ind2sub([ny nx], cur);
    for k = 1:8
        yy = cy + nbr(k,1);  xx = cx + nbr(k,2);
        if yy < 1 || yy > ny || xx < 1 || xx > nx, continue; end
        if blocked(yy,xx) || closed(yy,xx), continue; end
        if nbr(k,1) ~= 0 && nbr(k,2) ~= 0     % không cắt góc vật cản
            if blocked(cy+nbr(k,1), cx) || blocked(cy, cx+nbr(k,2)), continue; end
        end
        gn = G(cy,cx) + cost(k);
        if gn < G(yy,xx)
            G(yy,xx) = gn;  parent(yy,xx) = cur;
            openF(yy,xx) = gn + octile([yy xx], g);
        end
    end
end
if ~found, path = []; return; end

% dựng lại đường từ đích về xuất phát
idx = gIdx;  cells = zeros(0,2);
while idx ~= 0
    [cy, cx] = ind2sub([ny nx], idx);
    cells(end+1,:) = [cy cx]; %#ok<AGROW>
    idx = parent(cy,cx);
end
cells = flipud(cells);
P = [b0(1) + (cells(:,2)-0.5)*cs,  b0(3) + (cells(:,1)-0.5)*cs];

% làm trơn bằng line-of-sight
sm = P(1,:);  i = 1;  n = size(P,1);
while i < n
    j = n;
    while j > i+1 && ~los_ok(blocked, P(i,:), P(j,:), b0, cs, nx, ny)
        j = j - 1;
    end
    sm(end+1,:) = P(j,:); %#ok<AGROW>
    i = j;
end
path = sm;
if ~blocked_at(blocked, start, b0, cs, nx, ny), path(1,:)   = start; end
if ~blocked_at(blocked, goal,  b0, cs, nx, ny), path(end,:) = goal;  end
end

% ------------------------------------------------------------------------
function [blocked, nx, ny] = build_grid(map, cfg, tmp_obs)
b = map.bounds;  cs = cfg.cell;
nx = round((b(2)-b(1))/cs);  ny = round((b(4)-b(3))/cs);
xc = b(1) + ((1:nx)-0.5)*cs;  yc = b(3) + ((1:ny)-0.5)*cs;
[XC, YC] = meshgrid(xc, yc);
infl = cfg.r_robot + cfg.inflate_extra;
blocked = reshape(dist_static([XC(:) YC(:)], map) < infl, ny, nx);
for k = 1:size(tmp_obs,1)
    blocked = blocked | (sqrt((XC-tmp_obs(k,1)).^2 + (YC-tmp_obs(k,2)).^2) - tmp_obs(k,3) < infl);
end
end

function c = to_cell(p, b0, cs, nx, ny)
ix = min(max(floor((p(1)-b0(1))/cs)+1, 1), nx);
iy = min(max(floor((p(2)-b0(3))/cs)+1, 1), ny);
c = [iy ix];
end

function c = nearest_free(blocked, c)
if ~blocked(c(1), c(2)), return; end
[fy, fx] = find(~blocked);
[~, m] = min((fy-c(1)).^2 + (fx-c(2)).^2);
c = [fy(m) fx(m)];
end

function h = octile(a, g)
dy = abs(a(1)-g(1));  dx = abs(a(2)-g(2));
h = (dx + dy) + (sqrt(2) - 2)*min(dx, dy);
end

function tf = blocked_at(blocked, p, b0, cs, nx, ny)
c = to_cell(p, b0, cs, nx, ny);
tf = blocked(c(1), c(2));
end

function ok = los_ok(blocked, a, b, b0, cs, nx, ny)
n  = max(2, ceil(norm(b-a)/(cs/2)));
t  = linspace(0, 1, n)';
Q  = a + t.*(b-a);
ix = min(max(floor((Q(:,1)-b0(1))/cs)+1, 1), nx);
iy = min(max(floor((Q(:,2)-b0(3))/cs)+1, 1), ny);
ok = ~any(blocked(sub2ind([ny nx], iy, ix)));
end
