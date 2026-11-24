function d = dist_static(P, map, use_border)
%DIST_STATIC  Khoảng cách từ các điểm P (n x 2) tới MÉP vật cản tĩnh gần nhất
%   (kệ, cột, biên bản đồ). Giá trị = 0 nếu điểm nằm trong vật cản; âm nếu
%   ra ngoài biên (nếu use_border = true, mặc định). Tính giải tích nên không cần Image Processing Toolbox.
if nargin < 3, use_border = true; end
x = P(:,1);  y = P(:,2);
b = map.bounds;
if use_border
    d = min([x-b(1), b(2)-x, y-b(3), b(4)-y], [], 2);
else
    d = inf(size(x));            % chỉ xét kệ/cột (không coi biên bản đồ là vật cản)
end
for i = 1:size(map.rects,1)
    r  = map.rects(i,:);
    dx = max(max(r(1)-x, x-r(3)), 0);
    dy = max(max(r(2)-y, y-r(4)), 0);
    d  = min(d, sqrt(dx.^2 + dy.^2));
end
for i = 1:size(map.circles,1)
    c = map.circles(i,:);
    d = min(d, max(sqrt((x-c(1)).^2 + (y-c(2)).^2) - c(3), 0));
end
end
