# Xuat video demo cho S1-S5

## Ket qua

Moi script `run_S1.m` ... `run_S5.m` se xuat 3 video MP4 tu lan chay minh hoa (Phan A):

- DWA
- DWA-KF
- ORCA

Tong cong 15 video. Video duoc luu trong thu muc `videos/S1` ... `videos/S5`.

Vi du: `videos/S1/S1_DWA.mp4`, `videos/S1/S1_DWA_KF.mp4`, `videos/S1/S1_ORCA.mp4`.

## Cach chay

1. Giai nen toan bo project vao cung mot thu muc.
2. Mo MATLAB va chuyen Current Folder den thu muc project.
3. Chay mot trong cac script `run_S1.m` ... `run_S5.m`.
4. Sau khi tinh xong 3 lan chay minh hoa, MATLAB lan luot ghi 3 video vao thu muc `videos/<Sx>`.
5. Phan thong ke 50 seed van chay nhu cu; khong ghi video cho cac seed thong ke.

## Dinh dang

Su dung `VideoWriter(...,'MPEG-4')`, 30 FPS, quality 95. Moi buoc mo phong duoc lap frame de thoi luong video gan voi thoi gian mo phong (voi `cfg.dt = 0.1`).

Can MATLAB/nen tang co ho tro profile MPEG-4 cua `VideoWriter`. Trong luc ghi, mot cua so figure animation se hien thi va tu dong dong sau khi xuat xong.

## Cac thanh phan tren video

- Ban do va vat can tinh.
- Duong A* ban dau va duong A* cuoi cung (neu co tinh lai).
- Robot, huong robot va vet quy dao.
- Nguoi di bo va vet di chuyen (S2-S5).
- Vi tri replan (S5).
