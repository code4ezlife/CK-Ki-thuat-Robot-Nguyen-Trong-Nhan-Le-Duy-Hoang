function tracks = kalman_update(tracks, z, ids, cfg)
%KALMAN_UPDATE  Bộ lọc Kalman tuyến tính, mô hình vận tốc không đổi (2.6-2.10).
%   Trạng thái mỗi vật: x = [px py vx vy]'.  Mỗi chu kỳ: dự đoán -> cập nhật.
dt = cfg.dt;
F  = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
H  = [1 0 0 0; 0 1 0 0];
q  = cfg.sigma_a^2;
Q1 = q*[dt^4/4 dt^3/2; dt^3/2 dt^2];
Q  = zeros(4);  Q([1 3],[1 3]) = Q1;  Q([2 4],[2 4]) = Q1;
R  = cfg.sigma_z^2*eye(2);

for k = 1:numel(tracks)                      % bước dự đoán (2.8)
    tracks(k).x = F*tracks(k).x;
    tracks(k).P = F*tracks(k).P*F' + Q;
    tracks(k).unseen = tracks(k).unseen + dt;
end
for m = 1:size(z,1)                          % bước cập nhật (2.9-2.10)
    k = find([tracks.id] == ids(m), 1);
    if isempty(k)
        tracks(end+1) = struct('id',ids(m), 'x',[z(m,:)'; 0; 0], ...
                               'P',cfg.P0, 'unseen',0, 'z',z(m,:)); %#ok<AGROW>
    else
        S = H*tracks(k).P*H' + R;
        K = tracks(k).P*H'/S;
        tracks(k).x = tracks(k).x + K*(z(m,:)' - H*tracks(k).x);
        tracks(k).P = (eye(4) - K*H)*tracks(k).P;
        tracks(k).unseen = 0;
        tracks(k).z = z(m,:);
    end
end
if ~isempty(tracks)
    tracks([tracks.unseen] > cfg.track_timeout) = [];
end
end
