function v = orca_lp(L, vpref, vmax)
%ORCA_LP  Bài toán quy hoạch tuyến tính 2D của ORCA:
%   tìm v gần vpref nhất sao cho  n_i·v >= c_i (mọi ràng buộc)  và  |v| <= vmax.
%   L: m x 3, mỗi hàng [nx ny c].  Giải bằng liệt kê điểm ứng viên (m nhỏ, <= ~35):
%   hình chiếu của vpref lên từng biên, giao biên-đường tròn, giao từng cặp biên.
%   Nếu vô nghiệm: chọn điểm vi phạm lớn nhất là nhỏ nhất.
vp = vpref;
if norm(vp) > vmax, vp = vp*vmax/norm(vp); end
if isempty(L), v = vp; return; end
L(:,1:2) = L(:,1:2) ./ sqrt(sum(L(:,1:2).^2, 2));    % chuẩn hóa pháp tuyến
m = size(L,1);

C = [vp; 0 0];
C = [C; vp + (L(:,3) - L(:,1:2)*vp').*L(:,1:2)];     % hình chiếu lên từng biên
for i = 1:m                                          % giao biên - vòng tròn |v|=vmax
    c = L(i,3);
    if abs(c) <= vmax
        t = sqrt(vmax^2 - c^2);  dv = [-L(i,2) L(i,1)];
        C = [C; c*L(i,1:2) + t*dv; c*L(i,1:2) - t*dv]; %#ok<AGROW>
    end
end
if m >= 2                                            % giao từng cặp biên
    [I, J] = find(triu(true(m), 1));
    dt = L(I,1).*L(J,2) - L(I,2).*L(J,1);
    k  = abs(dt) > 1e-9;
    I = I(k); J = J(k); dt = dt(k);
    C = [C; [(L(I,3).*L(J,2) - L(J,3).*L(I,2))./dt, (L(I,1).*L(J,3) - L(J,1).*L(I,3))./dt]]; %#ok<AGROW>
end
C = C(sum(C.^2, 2) <= vmax^2 + 1e-9, :);

viol = max(L(:,3) - L(:,1:2)*C', [], 1)';            % <= 0 : thỏa mọi ràng buộc
feas = viol <= 1e-6;
if any(feas)
    Cf = C(feas,:);
else
    Cf = C(viol <= min(viol) + 1e-6, :);
end
[~, i] = min(sum((Cf - vp).^2, 2));
v = Cf(i,:);
end
