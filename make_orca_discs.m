function D = make_orca_discs(map, rd, spacing)
%MAKE_ORCA_DISCS  Xấp xỉ vật cản tĩnh bằng tập đĩa tròn phủ kín (mục 3.3.6).
%   Mỗi chữ nhật được phủ bằng các đĩa bán kính rd nằm hoàn toàn bên trong.
%   Đầu ra D: [x y r] cho từng đĩa (đã gồm cả các cột tròn).
if nargin < 2, rd = 0.20; end
if nargin < 3, spacing = 0.25; end   % (spacing/sqrt2 < rd) => không hở
D = zeros(0,3);
for i = 1:size(map.rects,1)
    r  = map.rects(i,:);
    xs = disc_centers(r(1)+rd, r(3)-rd, spacing);
    ys = disc_centers(r(2)+rd, r(4)-rd, spacing);
    [X,Y] = meshgrid(xs, ys);
    D = [D; X(:) Y(:) rd*ones(numel(X),1)]; %#ok<AGROW>
end
D = [D; map.circles];
end

function c = disc_centers(a, b, sp)
if b <= a
    c = (a+b)/2;
else
    c = linspace(a, b, ceil((b-a)/sp)+1);
end
end
