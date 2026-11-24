function cfg = config_params()
%CONFIG_PARAMS  Toàn bộ tham số mô phỏng (Bảng 3.2 trong báo cáo).
%   Thay đổi tham số ở MỘT chỗ duy nhất này.

% ---- Robot vi sai -------------------------------------------------------
cfg.r_robot = 0.25;      % bán kính robot [m]
cfg.b       = 0.4;       % khoảng cách hai bánh [m]
cfg.v_max   = 1.0;       % vận tốc dài tối đa [m/s]
cfg.w_max   = 1.5;       % vận tốc góc tối đa [rad/s]
cfg.a_v     = 1.0;       % gia tốc dài tối đa [m/s^2]
cfg.a_w     = 3.0;       % gia tốc góc tối đa [rad/s^2]

% ---- Mô phỏng -----------------------------------------------------------
cfg.dt       = 0.1;      % chu kỳ điều khiển [s]
cfg.T_max    = 60;       % thời gian tối đa [s]
cfg.goal_tol = 0.3;      % ngưỡng tới đích [m]

% ---- Cảm biến -----------------------------------------------------------
cfg.sense_range = 8;     % tầm quét [m]
cfg.sigma_z     = 0.1;   % độ lệch chuẩn nhiễu đo [m]

% ---- Người đi bộ (vật cản động; S1 không dùng) --------------------------
cfg.r_human = 0.3;       % bán kính [m]
cfg.sigma_h = 0.3;       % nhiễu gia tốc của người [m/s^2]
cfg.lambda  = 1.0;       % hệ số bám v_ref [1/s]
cfg.s4_nhum  = 6;            % S4: số người (Bảng 3.1: 6)
cfg.s5_human2 = true;        % S5: có người 2 (rẽ đột ngột) hay không (false = chỉ người 1 chặn cửa)
cfg.s3_width = 2.0;          % S3: bề rộng hành lang [m] (Bảng 3.1: 2 m)
cfg.s2_jitter = [-0.5 0.5];  % S2: lệch thời điểm người xuất hiện so với mốc "cùng tới giao điểm" [s]
                             % (đặt [0 1] nếu muốn đúng nghĩa đen "lệch 0-1 s")

% ---- Bộ lọc Kalman ------------------------------------------------------
cfg.sigma_a       = 0.5;                 % q = sigma_a^2
cfg.P0            = diag([0.01 0.01 1 1]);
cfg.track_timeout = 1.0;                 % xóa track nếu mất số đo > 1 s
cfg.T_pred  = 2.0;                       % chân trời dự đoán [s]
cfg.delta   = 0.1;                       % bước dự đoán/mô phỏng quỹ đạo [s]
cfg.n_pred  = round(cfg.T_pred/cfg.delta);   % = 20 bước

% ---- Planner toàn cục A* ------------------------------------------------
cfg.cell          = 0.1;   % ô lưới [m]
cfg.inflate_extra = 0.1;   % phồng bản đồ = r_robot + 0.1 [m]
cfg.lookahead     = 1.5;   % khoảng nhìn trước (pure pursuit) [m]: A*, ORCA (Bảng 3.2)
cfg.lookahead_dwa = 2.0;   % khoảng nhìn trước cho DWA / DWA-KF [m]. PHẢI >= v_max*T_pred = 2 m;
                           % nếu < 2 m, số hạng g (3.2) phạt vượt mục tiêu và khóa v_hành_trình = d/T
                           % (1,5 m -> 0,75 m/s). Đặt = 1.5 để tái hiện kết quả lần chạy đầu.
cfg.replan_gap    = 2.0;   % giãn cách tối thiểu giữa 2 lần tính lại [s]
cfg.T1_speed = 0.2;        % T1: vật "đứng yên" nếu tốc độ ước lượng < 0.2 m/s
cfg.T1_range = 4.0;        % T1: xét 4 m đường phía trước
cfg.T2_dist  = 0.2;        % T2: tiến < 0.2 m ...
cfg.T2_time  = 5.0;        %     ... trong 5 s => kẹt
cfg.T3_dev   = 1.0;        % T3: lệch đường > 1 m

% ---- DWA (chung cho DWA gốc và DWA-KF) ----------------------------------
cfg.nv = 11;  cfg.nw = 21;           % lưới lấy mẫu (v x w)
cfg.w_h = 0.5; cfg.w_g = 1.0; cfg.w_c = 0.8; cfg.w_v = 0.3;   % trọng số (3.2)
cfg.fallback = 'brake';              % khi không còn quỹ đạo hợp lệ: 'brake' (báo cáo 3.3.4: phanh + quay tại chỗ)
                                     %   hoặc 'least_violation' (chọn quỹ đạo vi phạm biên ít nhất, vẫn né)
cfg.d_c = 1.5;                       % chặn trên của clearance [m]
cfg.m   = 0.2;                       % biên an toàn với vật cản ĐỘNG [m]
cfg.m_static = 0.05;                 % biên an toàn với vật cản TĨNH [m] (xem ghi chú README)

% ---- DWA-KF -------------------------------------------------------------
cfg.kappa  = 2;          % hệ số phồng theo độ bất định
cfg.d_warn = 0.5;        % ngưỡng cảnh báo rủi ro [m]
cfg.w_r    = 0.6;        % trọng số số hạng rủi ro

% ---- ORCA ---------------------------------------------------------------
cfg.orca_tau    = 3.0;   % chân trời thời gian [s]
cfg.orca_neigh  = 5.0;   % bán kính xét láng giềng [m]
cfg.orca_Kw     = 2.0;   % hệ số quay K_w
cfg.orca_margin = 0.12;  % biên phụ cho xấp xỉ vật tĩnh bằng đĩa tròn [m]
cfg.orca_margin_dyn = 0.70;  % biên phụ cho vật cản ĐỘNG [m] (hiệu chỉnh trên hạt giống 1-20, xem tune_orca_S2.m)
cfg.orca_K      = 40;    % số đĩa tĩnh gần nhất được xét

% ---- Đánh giá -----------------------------------------------------------
cfg.n_runs      = 50;
cfg.seeds_tune  = 1:20;
cfg.seeds_eval  = 101:150;
cfg.near_miss   = 0.5;   % ngưỡng suýt va chạm [m]
end
