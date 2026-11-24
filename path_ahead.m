function [pts, dev] = path_ahead(path, pos, L, step)
%PATH_AHEAD  Chiếu pos lên đường gấp khúc rồi lấy các điểm cách nhau 'step'
%   dọc đường, tới tối đa L mét phía trước. dev = khoảng cách robot-đường.
if size(path,1) == 1
    pts = path;  dev = norm(pos - path);  return
end
best = Inf; bi = 1; bc = path(1,:);
for i = 1:size(path,1)-1
    a = path(i,:);  b = path(i+1,:);  ab = b - a;  l2 = ab*ab';
    if l2 < 1e-12, tt = 0; else, tt = max(0, min(1, ((pos-a)*ab')/l2)); end
    c = a + tt*ab;  d = norm(pos - c);
    if d < best, best = d; bi = i; bc = c; end
end
dev = best;
V   = [bc; path(bi+1:end,:)];
seg = sqrt(sum(diff(V,1,1).^2, 2));
cum = [0; cumsum(seg)];
tot = cum(end);
sv  = unique(min([step:step:L, L], tot));
pts = zeros(numel(sv), 2);
for k = 1:numel(sv)
    s = sv(k);
    j = find(cum <= s, 1, 'last');
    if j >= numel(cum)
        pts(k,:) = V(end,:);
    else
        tt = (s - cum(j)) / seg(j);
        pts(k,:) = V(j,:) + tt*(V(j+1,:) - V(j,:));
    end
end
end
