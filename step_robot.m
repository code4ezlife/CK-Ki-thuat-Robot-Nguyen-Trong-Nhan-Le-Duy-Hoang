function robot = step_robot(robot, v_c, w_c, cfg)
%STEP_ROBOT  Động học robot vi sai (2.1, 2.4) có giới hạn gia tốc và vận tốc.
dt = cfg.dt;
v = min(max(v_c, robot.v - cfg.a_v*dt), robot.v + cfg.a_v*dt);
w = min(max(w_c, robot.w - cfg.a_w*dt), robot.w + cfg.a_w*dt);
v = min(max(v, 0), cfg.v_max);
w = min(max(w, -cfg.w_max), cfg.w_max);
robot.x  = robot.x + v*cos(robot.th)*dt;
robot.y  = robot.y + v*sin(robot.th)*dt;
robot.th = robot.th + w*dt;
robot.v  = v;  robot.w = w;
end
