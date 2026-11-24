function a = wrapPi(a)
%WRAPPI  Chuẩn hóa góc về [-pi, pi] (thay cho angdiff; xem mục 2.2.1).
a = atan2(sin(a), cos(a));
end
