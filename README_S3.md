# Kịch bản S3 – Ngược chiều trong hành lang hẹp (MATLAB)

Hành lang dài 12 m, hai bên là tường (bề rộng: xem "Cấu hình đang dùng" bên dưới; Bảng 3.1 ghi 2 m). Robot đi (1;6)→(11;6). Một người đi ngược chiều từ (10;6)
về (1;6) với 1,0 ± 0,2 m/s, xuất hiện lệch 0–1 s, **không nhường đường**.

## Chạy
`run_S3` (Phần A: minh họa + 4 hình; Phần B: 50 hạt giống + bảng + McNemar/Wilcoxon). `quick = true` để thử 5 hạt giống.
`sweep_S3` (tùy chọn): tỉ lệ va chạm theo bề rộng hành lang × cơ chế dự phòng, trên hạt giống 1–20.

## Tệp mới / đổi
`scenario_S3.m`, `run_S3.m`, `plot_results_S3.m`, `sweep_S3.m` (mới).
`sim_main.m` (ghi thêm `d_human`), `compute_metrics.m` (thêm `d_min_h`, `near_miss_h`: khoảng cách/suýt va chạm riêng với NGƯỜI,
vì tường hành lang là vật cản thật nên `d_min` chung bị tường chi phối), `summarize_results.m`, `plot_results_S2.m` (dùng chung),
`config_params.m` (`s3_width`, `fallback`), `dwa_original.m`, `dwa_kf.m` (tùy chọn `fallback`).

## Kết quả thử với ĐÚNG mô tả báo cáo (2 m, fallback = 'brake')
Cả ba thuật toán va chạm ~100 % (Octave, 12 hạt giống). Đây là đặc tính của kịch bản + tham số, không phải lỗi mô phỏng:
- Tâm robot phải cách người ≥ r_h + r_robot + m = 0,3 + 0,25 + 0,2 = 0,75 m (DWA), nhiều hơn nữa với κσ (DWA-KF) và 1,25 m (ORCA, biên động 0,7 m).
- Tâm robot phải cách tường ≥ 0,3 m ⇒ dải tâm robot trong hành lang 2 m chỉ rộng 1,4 m (y = 5,3..6,7); người ở giữa (y ≈ 6) ⇒
  khoảng tách tối đa chỉ 0,7 m < 0,75 m: **không còn quỹ đạo hợp lệ nào để vượt qua**.
- Khi hết quỹ đạo hợp lệ, cơ chế của báo cáo (phanh + quay tại chỗ về phía trống tĩnh) làm robot đứng yên, người không nhường đi thẳng vào.

## Hai công tắc để làm S3 giải được (cần bạn quyết định, ghi rõ trong báo cáo)
| Công tắc | Ý nghĩa | Ảnh hưởng (Octave, 12 hạt giống, tỉ lệ va chạm DWA/DWA-KF/ORCA) |
|---|---|---|
| `cfg.s3_width` | Bề rộng hành lang | 2,0 m: 100/100/100 % · 2,8 m: 100/100/83 % · 3,2 m: 100/100/67 % |
| `cfg.fallback = 'least_violation'` | Khi hết quỹ đạo hợp lệ vẫn né theo quỹ đạo vi phạm biên ít nhất | 2,0 m: DWA-KF 92 % · 3,2 m: DWA-KF 33 % (DWA gốc vẫn 100 %) |

## Cấu hình đang dùng trong `run_S3.m` (khác báo cáo - PHẢI ghi rõ trong Chương 3/4)
- `cfg.s3_width = 4.0` m (báo cáo: 2 m) và `cfg.fallback = 'least_violation'` (báo cáo 3.3.4: phanh + quay tại chỗ).
- Chọn trên tập hiệu chỉnh (hạt giống 1–20): 4,0 m là bề rộng hẹp nhất để ORCA 0/20 va chạm và DWA-KF 1/20
  (3,2 m: ORCA 9/20; DWA gốc ~18/20 ở mọi bề rộng vì coi người đứng yên). Sau đó đánh giá trên 101–150, không chỉnh thêm.
- Muốn tái hiện đúng báo cáo: đặt `s3_width = 2.0`, `fallback = 'brake'` (kết quả ~100 % va chạm cả ba).

## Kết quả đánh giá (Octave, hạt giống 101–150; số trên MATLAB sẽ lệch nhẹ)
| | DWA | DWA-KF | ORCA |
|---|---|---|---|
| Va chạm | 100 % | 10 % (5/50) | 10 % (5/50) |
| T [s] | – | 22,3 ± 2,7 | 13,5 ± 1,1 |
| L_ratio | – | 1,48 ± 0,17 | 1,04 ± 0,03 |
| d_min_h [m] (tới người) | – | 0,58 | 0,61 |
| near_miss_h | – | 0,40 | 0,07 |
| mean_jerk | – | 2,48 | 1,26 |
| reversals | – | 1,3 ± 1,4 | 4,5 ± 2,2 |
McNemar: DWA-KF vs DWA p ≈ 6e-14; DWA-KF vs ORCA p = 1 (5 so với 5, không khác biệt).
