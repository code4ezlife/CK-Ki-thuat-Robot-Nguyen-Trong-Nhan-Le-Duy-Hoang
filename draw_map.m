function draw_map(map)
%DRAW_MAP  Vẽ bản đồ tĩnh (kệ chữ nhật + cột tròn) lên trục hiện tại.
hold on;  axis equal;  box on;
xlim(map.bounds(1:2));  ylim(map.bounds(3:4));
for i = 1:size(map.rects,1)
    r = map.rects(i,:);
    rectangle('Position',[r(1) r(2) r(3)-r(1) r(4)-r(2)], ...
              'FaceColor',[0.6 0.6 0.6], 'EdgeColor',[0.2 0.2 0.2]);
end
for i = 1:size(map.circles,1)
    c = map.circles(i,:);
    rectangle('Position',[c(1)-c(3) c(2)-c(3) 2*c(3) 2*c(3)], 'Curvature',[1 1], ...
              'FaceColor',[0.7 0.7 0.7], 'EdgeColor',[0.2 0.2 0.2]);
end
end
