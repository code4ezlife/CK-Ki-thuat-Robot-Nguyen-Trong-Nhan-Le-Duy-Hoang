function [trig, tmp_obs, why] = replan_trigger(path, tracks, robot, pos_hist, goal, t, t_last, cfg)
%REPLAN_TRIGGER  Điều kiện tính lại đường đi (mục 3.3.3):
%   T1 - đường bị chặn bởi vật cản gần như đứng yên (< 0.2 m/s) trong 4 m phía trước
%   T2 - kẹt: tiến < 0.2 m trong 5 s gần nhất
%   T3 - lệch khỏi đường toàn cục > 1.0 m
%   Giãn cách tối thiểu 2 s giữa hai lần tính lại.
trig = false;  tmp_obs = zeros(0,3);  why = '';
if (t - t_last) < cfg.replan_gap, return; end
pos = [robot.x robot.y];
[pts, dev] = path_ahead(path, pos, cfg.T1_range, 0.25);

if dev > cfg.T3_dev                                        % T3
    trig = true;  why = 'T3';
end
if t >= cfg.T2_time && norm(pos - goal) > cfg.goal_tol     % T2
    i = find(pos_hist(:,1) <= t - cfg.T2_time, 1, 'last');
    if ~isempty(i) && norm(pos - pos_hist(i,2:3)) < cfg.T2_dist
        trig = true;  why = 'T2';
    end
end
for j = 1:numel(tracks)                                    % T1
    if norm(tracks(j).x(3:4)) < cfg.T1_speed
        pj = tracks(j).x(1:2)';
        if min(sqrt(sum((pts - pj).^2, 2))) < cfg.r_human + cfg.r_robot + cfg.m
            tmp_obs(end+1,:) = [pj cfg.r_human]; %#ok<AGROW>
            trig = true;  why = 'T1';
        end
    end
end
end
