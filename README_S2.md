# Kịch bản S2 – Một người cắt ngang (MATLAB)

Không gian trống 12×12 m. Robot đi (1;6)→(11;6). Một người đi (6;0,8)→(6;11) với ~1,2 m/s và
**tới giao điểm cùng lúc với robot**. So sánh DWA / DWA-KF / ORCA (cùng A\*, cảm biến, Kalman, hạt giống).

## Cách chạy
1. Giải nén **toàn bộ** thư mục (S1 và S2 dùng chung các hàm), `cd` vào thư mục đó.
2. `run_S2` – Phần A: 1 lần minh họa + 3 hình; Phần B: 50 hạt giống (101–150), bảng thống kê + Wilcoxon.
3. `sensitivity_S2` – khảo sát độ nhạy DWA-KF: κ = 0,1,2,3 và σ_z = 0,05/0,1/0,2 m.
4. `tune_orca_S2` – (tùy chọn) hiệu chỉnh biên an toàn động của ORCA trên hạt giống 1–20.
   `quick = true` trong các script để thử nhanh với 5 hạt giống.

## Tệp mới / thay đổi so với S1 (nếu bạn đã có thư mục S1, chép đè)
| Tệp | Nội dung |
|---|---|
| `scenario_S2.m` (mới) | Bản đồ trống, robot, người, đồng bộ thời điểm tới giao điểm |
| `run_S2.m`, `sensitivity_S2.m`, `tune_orca_S2.m` (mới) | Chạy, độ nhạy, hiệu chỉnh ORCA |
| `plot_results_S2.m`, `summarize_results.m` (mới) | Hình S2 (quỹ đạo robot–người, v/ω/d_edge, Kalman); bảng thống kê |
| `sim_main.m`, `sense.m`, `step_humans.m` (đổi) | Nhiễu cảm biến/chuyển động người sinh sẵn từ hạt giống (3 thuật toán cùng điều kiện); ghi nhật ký người + Kalman |
| `config_params.m`, `orca_step.m`, `scenario_S1.m` (đổi nhỏ) | Thêm `s2_jitter`, `orca_margin_dyn`; `scn.seed` |

## Cách mô hình hóa "tới giao điểm cùng lúc" (mục 3.3.7)
Với mỗi hạt giống, tính thời gian lý tưởng robot tới x = 6 (tăng tốc a_v rồi chạy v_max) và thời gian người tới
điểm cắt (y robot tại x = 6) với tốc độ 1,2 ± 0,2 m/s. Người xuất hiện lúc `t_start = t_robot − t_người + jitter`.
**Jitter mặc định ∈ [−0,5; 0,5] s** (cửa sổ rộng 1 s quanh mốc "cùng lúc"); nếu muốn đúng nghĩa đen "lệch 0–1 s"
đặt `cfg.s2_jitter = [0 1]` (khi đó người thường đi qua trước robot, ít xung đột hơn).

## Lưu ý khi diễn giải
- ORCA được hiệu chỉnh **một** tham số: biên an toàn vật cản động `orca_margin_dyn` (0,12 → 0,7 m, chọn trên hạt giống 1–20).
- ORCA dùng bản thích nghi đơn giản (không xét giới hạn gia tốc, người không nhường) nên có thể va chạm; không nên suy ra "ORCA kém" nói chung.

## Cập nhật sau lần chạy đầu trên MATLAB
- **Sửa phép đo d_min** (`dist_static.m`, `sim_main.m`): biên bản đồ không còn bị tính là vật cản khi đo d_min / suýt va chạm
  (chỉ tính va chạm nếu robot đi ra ngoài bản đồ). Trước đó, ở S2 d_min bị chặn trên bởi khoảng cách tới tường trái (~0,75 m) ở t = 0.
  **Chạy lại `run_S2` (và `run_S1` cho đồng bộ).**
- **Khoảng nhìn trước riêng cho DWA/DWA-KF: `cfg.lookahead_dwa = 2.0 m`** (= v_max·T). Với d = 1,5 m và chân trời T = 2 s, số hạng
  tiến triển `g` trong (3.2) phạt vượt mục tiêu cục bộ nên khóa tốc độ hành trình ở d/T = 0,75 m/s (ORCA vẫn chạy 1,0 m/s) và che mất
  việc DWA gốc không né được người. ORCA, A* giữ `cfg.lookahead = 1,5 m` (Bảng 3.2). Đặt `lookahead_dwa = 1.5` để tái hiện lần chạy đầu.
  Cần ghi rõ thay đổi này trong Bảng 3.2; `lookahead_S2.m` cho bảng khảo sát d = 1,5 / 2,0 / 2,5 m.
- Đã thử tách ràng buộc cứng/mềm của κσ trong DWA-KF: làm va chạm tăng (1/25 → 6–11/25) nên KHÔNG áp dụng; DWA-KF giữ đúng công thức (3.3).
