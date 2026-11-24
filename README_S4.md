# Kịch bản S4 – Nhiều người di chuyển (MATLAB)

Không gian trống 12×12 m. Robot đi (1;1)→(11;11). **6 người** (`cfg.s4_nhum`) cắt ngang bản đồ: mỗi người xuất phát cách biên 0,8 m ở một
cạnh ngẫu nhiên, đi tới điểm ngẫu nhiên ở cạnh đối diện (đích nằm ngoài bản đồ 1,5 m để người ra khỏi cảnh), tốc độ U(0,8; 1,4) m/s,
xuất hiện lệch 0–1 s, không né robot. Điểm xuất phát của người cách điểm đầu robot ≥ 3 m, cách đích ≥ 2 m, cách nhau ≥ 1 m.

## Chạy
- `run_S4` – minh họa + 4 hình (quỹ đạo, v/w/d, Kalman, thời gian tính) + 50 hạt giống (101–150) + bảng + McNemar/Wilcoxon.
- `scaling_S4` – tỉ lệ va chạm, d_min_h và thời gian tính theo số người 1, 2, 4, 6, 8, 10 (10 hạt giống mỗi mức).
- `sensitivity_S4` – độ nhạy DWA-KF theo κ = 0,1,2,3 và σ_z = 0,05/0,1/0,2 m (mục 3.3.8 yêu cầu trên S2 và S4).
`quick = true` trong `run_S4.m` / `sensitivity_S4.m` để thử nhanh 5 hạt giống.

## Tệp mới
`scenario_S4.m`, `run_S4.m`, `plot_results_S4.m`, `scaling_S4.m`, `sensitivity_S4.m`; `config_params.m` thêm `s4_nhum`.

## Kết quả thử (Octave, hạt giống 101–150; số trên MATLAB sẽ lệch nhẹ)
| | DWA | DWA-KF | ORCA |
|---|---|---|---|
| Va chạm | 14 % | 8 % | 0 % |
| d_min_h [m] | 0,66 | 1,07 | 0,79 |
| near_miss_h | 0,44 | 0,20 | 0,06 |
| T [s] | 17,6 | 19,4 | 16,7 |
| Thời gian tính [ms/chu kỳ] | 1,05 | 1,72 | 0,63 |
McNemar (va chạm): DWA-KF vs DWA 6 so với 3, p = 0,51; DWA-KF vs ORCA 0 so với 4, p = 0,12 → **không khác biệt có ý nghĩa**.
Kỳ vọng Bảng 3.1 ("chênh lệch an toàn rõ nhất") chỉ đúng một phần: d_min_h và near_miss_h của DWA-KF tốt hơn DWA gốc, nhưng tỉ lệ va chạm chưa phân biệt được.

## Số người tăng (scaling, 10 hạt giống, Octave)
Va chạm trên 10 lượt (DWA / DWA-KF / ORCA): 1 người 1/0/0 · 4 người 3/0/0 · 8 người 5/1/0 · 14 người 5/1/0.
Thời gian tính tăng chậm (DWA-KF 0,87 → 1,59 ms từ 1 → 14 người), xa dưới chu kỳ 100 ms.

## Lưu ý
- 4 trong 4 lượt DWA-KF va chạm (101–150) đều ở trạng thái **robot đứng yên (v = 0)** trong lối đi của người, dù người được theo dõi từ ~8 m:
  cơ chế dự phòng 'brake' (báo cáo 3.3.4) làm robot dừng khi hết quỹ đạo hợp lệ. `cfg.fallback = 'least_violation'` tránh được cả 4 lượt đó,
  và trên tập độc lập 201–250 cho DWA-KF 0/50 (so với 1/50 với 'brake'), nhưng chênh lệch nhỏ nên mặc định vẫn là 'brake'.
- Tỉ lệ va chạm dao động theo tập hạt giống (DWA 14–30 %, DWA-KF 0–8 %, ORCA 0–2 % giữa 101–150 và 201–250): đừng diễn giải chênh lệch vài lượt.
- ORCA dùng biên động 0,7 m hiệu chỉnh trên S2.
