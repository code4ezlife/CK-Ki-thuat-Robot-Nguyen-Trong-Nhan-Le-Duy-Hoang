function g = lookahead_point(path, pos, d)
%LOOKAHEAD_POINT  Điểm mục tiêu cục bộ kiểu pure pursuit (mục 2.2.4):
%   điểm nằm trước điểm gần robot nhất trên đường một khoảng d.
pts = path_ahead(path, pos, d, d);
g   = pts(end,:);
end
