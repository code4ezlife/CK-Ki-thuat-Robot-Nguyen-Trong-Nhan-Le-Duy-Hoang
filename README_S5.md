# Kịch bản S5 – Đường bị chặn và đổi hướng đột ngột (MATLAB)

Tường dọc x = 6 có **cửa rộng 1,6 m** (y = 5,2–6,8) và **lối vòng** phía trên (y = 10,5–11,8, rộng 1,3 m, dài hơn). Robot đi (1;6)→(11;6).
- **Người 1:** từ (10,5;6) đi về cửa rồi **dừng hẳn** ở (6,6;6) → chặn cửa (kích hoạt T1 → A\* tính lại, đi vòng).
- **Người 2:** từ (3; y0) đi thẳng xuống, **rẽ 90° vào đường robot ở giây thứ 6** (đổi ngay sang hướng đông, đi dọc đường đi ban đầu của robot, y ≈ 6, tới x = 4,3), rồi **rẽ nam rời khỏi vùng robot** (không đứng yên chặn đường). Dự đoán vận tốc không đổi của Kalman bị sai ngay sau lúc rẽ. Người 2 không đi qua cửa vì cửa đã bị người 1 chặn (người 1 đứng giữa cửa, còn hai khe 0,5 m mỗi bên).
Ngẫu nhiên theo hạt giống: xuất phát robot ±0,5 m; thời điểm xuất hiện của từng người lệch 0–1 s; tốc độ 1,0 ± 0,2 (người 1), 0,8 ± 0,2 (người 2); y của đường rẽ lệch ±0,2 m quanh đường đi của robot.

## Chạy
`run_S5` (minh họa + 3 hình: quỹ đạo/đường A\*/vị trí tính lại, v-w-d, sai số dự đoán của Kalman với người 2; rồi 50 hạt giống + bảng + McNemar). `quick = true` để thử 5 hạt giống.
Tùy chọn trong `run_S5.m`: `cfg.fallback` ('least_violation' hoặc 'brake'), `cfg.s5_human2 = false` (đối chứng không có người 2).
ORCA có thể kẹt đến hết 60 s nên một số lượt chạy lâu hơn S1–S4.

## Tệp mới / đổi
`scenario_S5.m`, `run_S5.m`, `plot_results_S5.m`; `step_humans.m` (rẽ đột ngột `turn_t`/`turn_goal` có tọa độ NaN = giữ nguyên, và `exit_goal` để rời đi);
`config_params.m` (`s5_human2`); `sim_main.m` (ghi `replan_t`, `replan_pos`); `compute_metrics.m` (`t_replan1`); `summarize_results.m` (`timeout%`, `n_replan`, `t_replan1`).

## Cấu hình và kết quả (Octave, hạt giống 101–150; số trên MATLAB sẽ lệch nhẹ)
`run_S5.m` dùng `cfg.fallback = 'least_violation'` (báo cáo 3.3.4: 'brake'), chọn trên hạt giống 1–20 (DWA-KF: 13/20 → 5/20 va chạm).
Lý do: với 'brake', DWA-KF dừng hẳn (v = 0) ngay trong lối đi của người 2 (từ ~4 s, trước cả lúc rẽ) rồi bị đâm — cùng cơ chế "bẫy dừng" như S3/S4.

| | DWA | DWA-KF | ORCA |
|---|---|---|---|
| Va chạm | 6 % | 8 % | 32 % |
| Thành công | 82 % | 88 % | 68 % |
| Hết giờ | 12 % | 4 % | 0 % |
| d_min_h [m] | 0,51 | 0,99 | 0,63 |
| near_miss_h | 0,73 | 0,18 | 0,41 |
| T [s] | 26,8 | 31,5 | 23,5 |
| L_ratio | 1,74 | 2,51 | 1,65 |
| Số lần tính lại | 2,5 | 4,7 | 1,6 |

McNemar (va chạm): DWA-KF vs DWA 3 so với 4 cặp lệch, p = 1 (không khác biệt); DWA-KF vs ORCA 15 so với 3 cặp, p = 0,0075.

Cùng 50 hạt giống, **fallback = 'brake'** (đúng báo cáo): va chạm DWA 6 % / DWA-KF **48 %** / ORCA 32 %; thành công 82 / 50 / 68 %.
**Đối chứng không có người 2** (`cfg.s5_human2 = false`, 'brake'): va chạm DWA 4 % / DWA-KF 0 % / ORCA 0 %; thành công 78 / 96 / 100 %; DWA-KF d_min_h = 1,87 m.
⇒ Phần lớn khó khăn của S5 đến từ người 2 (cắt vào đường robot đang chờ trước cửa), không phải từ cửa bị chặn.

## Lưu ý khi diễn giải
- DWA-KF giữ khoảng cách tới người lớn nhất và ít suýt va chạm nhất, nhưng **đi vòng rất dài** (L_ratio 2,5) vì tính lại đường nhiều lần.
- DWA gốc ít va chạm ở S5 nhưng d_min_h thấp hơn và 12 % hết giờ; DWA-KF so với DWA gốc về va chạm không khác biệt có ý nghĩa.
- Tỉ lệ va chạm của DWA-KF phụ thuộc mạnh vào cơ chế dự phòng (48 % → 8 %): phải ghi rõ cấu hình dùng.
- Một phần va chạm không thể tránh vì người 2 đổi hướng đột ngột cách robot ~1 m.
- ORCA 32 % va chạm: bán kính an toàn động 0,7 m (hiệu chỉnh trên S2) và kẹt gần cửa; không nên suy ra "ORCA kém" nói chung.
- Người không xét tường và không né nhau; người 1 và người 2 không tương tác với nhau.
