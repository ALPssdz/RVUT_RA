"""
RFUAV Dataset Builder v2.0 - Physics-Aligned Edition
======================================================
【核心设计原则】：训练图片的每一个物理参数必须与实际接收系统 (rf_stage2_waterfall_yolo.py) 完全对齐。
任何参数偏离都会导致"训练-部署域漂移（Domain Gap）"，使 YOLO 在实战中因看到陌生风格的图片而失效。

物理参数对齐矩阵（与 RF_Stage2_Dwell.generate_waterfall_tensor() 严格镜像）：
  - n_fft         = 4096   (vs 旧版 1024，分辨率提升 4x)
  - vmin          = -60 dB (vs 旧版 -70 dB，色彩映射起点统一)
  - vmax          = +30 dB (保持一致)
  - 色彩渲染器    = cv2.COLORMAP_HOT (vs 旧版 matplotlib hot，消除色彩空间差异)
  - 采样率        = 40MSPS (vs 旧版 100MSPS，通过 resample_poly 2:5 精准降采样)
  - 滑动窗口提帧  = 50% 重叠 (vs 旧版每文件只生成 1 帧)
  - 噪声增强      = AWGN SNR [10, 30] dB 随机注入 (仿真宿舍 EMC 复杂底噪环境)
"""

import os
import glob
import numpy as np
import cv2
import random
from scipy.signal import resample_poly


class RFUAV_DatasetBuilder_v2:
    def __init__(self,
                 root_dir="e:/Myprojects/RF-Vision-UAV-Tracker/Drone RF Data",
                 output_dir="e:/Myprojects/RF-Vision-UAV-Tracker/rf_yolo_dataset"):

        self.root_dir = root_dir
        self.output_dir = output_dir

        self.img_train_dir = os.path.join(output_dir, "images/train")
        self.img_val_dir   = os.path.join(output_dir, "images/val")
        self.lbl_train_dir = os.path.join(output_dir, "labels/train")
        self.lbl_val_dir   = os.path.join(output_dir, "labels/val")

        for d in [self.img_train_dir, self.img_val_dir, self.lbl_train_dir, self.lbl_val_dir]:
            os.makedirs(d, exist_ok=True)

        # 扫描类别（忽略非目录文件如 paper.md）
        self.classes = sorted([
            n for n in os.listdir(self.root_dir)
            if os.path.isdir(os.path.join(self.root_dir, n))
        ])
        print(f"[Builder] 侦测到 {len(self.classes)} 种无人机模型: {self.classes}")

        # =========================================================
        # 【物理参数硬锁区：严格与 rf_stage2_waterfall_yolo.py 镜像对齐】
        # =========================================================
        self.fft_size     = 4096     # 与 RF_Stage2_Dwell.fft_size 完全一致
        self.target_w     = 640      # YOLO 输入宽度
        self.target_h     = 640      # YOLO 输入高度
        self.vmin         = -60.0    # 与 RF_Stage2_Dwell.vmin 完全一致
        self.vmax         = 30.0     # 与 RF_Stage2_Dwell.vmax 完全一致
        self.orig_fs      = 100e6    # 原始数据集采样率 (来自 pack2.xml: SampleRate=100MSPS)
        self.target_fs    = 40e6     # PlutoSDR 实际接收采样率

        # 滑动窗口参数：每窗口对应 target_h 行 FFT 所需样本数
        # 在 40MSPS 下: samples_per_frame = 640 rows x 4096 fft_size = 2,621,440 samples
        # 在 100MSPS 原始域: 反算 = 2,621,440 x (100/40) = 6,553,600 samples/frame
        self.samples_per_frame_40m = self.target_h * self.fft_size  # 2,621,440
        self.samples_per_frame_100m = int(self.samples_per_frame_40m * (self.orig_fs / self.target_fs))

        # 50% 重叠滑动步长
        self.slide_step_100m = self.samples_per_frame_100m // 2

        # 布莱克曼窗（与 Stage2 一致）
        self.window = np.blackman(self.fft_size)

        # 频域池化比率 (4096 -> 640)
        self.pool_size = self.fft_size // self.target_w   # = 6
        self.trim_side = (self.fft_size - self.pool_size * self.target_w) // 2  # = 128

        print(f"[Builder] 物理参数锁定完毕:")
        print(f"  FFT size       = {self.fft_size}")
        print(f"  vmin / vmax    = {self.vmin} / {self.vmax} dB")
        print(f"  采样降频       = {int(self.orig_fs/1e6)}MSPS -> {int(self.target_fs/1e6)}MSPS")
        print(f"  每帧采样数     = {self.samples_per_frame_100m:,} (100MHz域) -> {self.samples_per_frame_40m:,} (40MHz域)")
        print(f"  50% 重叠步长   = {self.slide_step_100m:,} samples")

    def _iq_to_waterfall_bgr(self, iq_40m):
        """
        将已降采样至 40MSPS 的 IQ 数组转换为 YOLO 训练用 BGR 瀑布图。
        此函数的实现逻辑与 rf_stage2_waterfall_yolo.py 的 generate_waterfall_tensor() 完全一致。
        """
        # 直流抑制
        iq_40m = iq_40m - np.mean(iq_40m)

        # 裁剪/补齐
        valid_length = self.target_h * self.fft_size
        if len(iq_40m) > valid_length:
            iq_40m = iq_40m[:valid_length]
        elif len(iq_40m) < valid_length:
            iq_40m = np.pad(iq_40m, (0, valid_length - len(iq_40m)))

        # 向量化 FFT (完全镜像 Stage2)
        reshaped = iq_40m.reshape((self.target_h, self.fft_size))
        windowed = reshaped * self.window
        fft_data = np.fft.fftshift(np.fft.fft(windowed, axis=1), axes=1)
        power_db = 20 * np.log10(np.abs(fft_data) + 1e-12)

        # 频域最大值池化压缩 4096 -> 640
        trimmed = power_db[:, self.trim_side : self.trim_side + self.pool_size * self.target_w]
        pooled  = trimmed.reshape((self.target_h, self.target_w, self.pool_size))
        waterfall = np.max(pooled, axis=2)

        # 归一化 + OpenCV HOT 颜色映射（与 Stage2 完全一致！）
        clipped   = np.clip(waterfall, self.vmin, self.vmax)
        normed    = ((clipped - self.vmin) / (self.vmax - self.vmin) * 255.0).astype(np.uint8)
        bgr_frame = cv2.applyColorMap(normed, cv2.COLORMAP_HOT)

        return bgr_frame

    def _add_awgn(self, iq_40m, snr_db):
        """
        向路IQ数据注入加性高斯白噪声（AWGN），仿真真实宿舍 EMC 背景环境。
        公式：N_sigma = sqrt(P_signal / (10^(SNR/10)))
        """
        sig_power = np.mean(np.abs(iq_40m) ** 2)
        noise_power = sig_power / (10 ** (snr_db / 10.0))
        noise = np.sqrt(noise_power / 2.0) * (
            np.random.randn(len(iq_40m)) + 1j * np.random.randn(len(iq_40m))
        )
        return iq_40m + noise.astype(np.complex64)

    def process_iq_file(self, file_path, img_dir, lbl_dir, file_tag, class_id):
        """
        从单个 .iq 文件中以 50% 重叠滑动窗口提取多帧瀑布图。
        对每帧做多次噪声增强，大幅扩充数据集体量。
        返回生成的帧数。
        """
        print(f"  [>>] 读取: {os.path.basename(file_path)}")

        # 读取原始 complex64 数据 (100MSPS)
        raw = np.fromfile(file_path, dtype=np.complex64)
        total = len(raw)
        generated = 0

        snr_levels = [30, 20, 12]  # 三种 SNR：高中低，仿真不同距离下的信号强度

        for start in range(0, total - self.samples_per_frame_100m, self.slide_step_100m):
            chunk_100m = raw[start : start + self.samples_per_frame_100m]

            # [关键步骤] 100MSPS -> 40MSPS 精确降采样（镜像 PlutoSDR ADC 视角）
            chunk_40m = resample_poly(chunk_100m, 2, 5).astype(np.complex64)

            for snr_idx, snr_db in enumerate(snr_levels):
                # 注入 AWGN 仿真不同环境
                augmented = self._add_awgn(chunk_40m, snr_db)

                bgr = self._iq_to_waterfall_bgr(augmented)

                frame_name = f"uav_{class_id}_{file_tag}_s{start//1000000}_n{snr_idx}"
                img_path = os.path.join(img_dir, frame_name + ".jpg")
                lbl_path = os.path.join(lbl_dir, frame_name + ".txt")

                cv2.imwrite(img_path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])

                # 全图框（正样本：全频段范围均为目标信号）
                with open(lbl_path, "w") as f:
                    f.write("0 0.5 0.5 1.0 1.0\n")

                generated += 1

        return generated

    def build(self):
        print("\n[Builder] === 开始构建物理对齐数据集 v2.0 ===\n")
        total_count = 0

        for class_id, class_name in enumerate(self.classes):
            iq_files = glob.glob(
                os.path.join(self.root_dir, class_name, "**", "*.iq"), recursive=True
            )
            print(f"\n[{class_name}] (class_id={class_id}): 找到 {len(iq_files)} 个 .iq 文件")

            for iq_file in iq_files:
                is_train = random.random() < 0.85  # 85% 训练 / 15% 验证
                img_dir = self.img_train_dir if is_train else self.img_val_dir
                lbl_dir = self.lbl_train_dir if is_train else self.lbl_val_dir
                file_tag = os.path.splitext(os.path.basename(iq_file))[0].replace("-", "_")

                n = self.process_iq_file(iq_file, img_dir, lbl_dir, file_tag, class_id)
                total_count += n
                print(f"       生成 {n} 帧 (累计: {total_count})")

        print(f"\n[Builder] === 数据集构建完毕！共生成 {total_count} 帧训练图片 ===")
        print(f"  训练集: {self.img_train_dir}")
        print(f"  验证集: {self.img_val_dir}")

        yaml_path = os.path.join(self.output_dir, "rf_uav.yaml")
        with open(yaml_path, "w") as f:
            f.write(f"train: {self.img_train_dir.replace(chr(92), '/')}\n")
            f.write(f"val:   {self.img_val_dir.replace(chr(92), '/')}\n\n")
            f.write("nc: 1\n")
            f.write("names: ['UAV_Signal']\n")
        print(f"[Builder] YAML 配置已写入: {yaml_path}")


if __name__ == "__main__":
    builder = RFUAV_DatasetBuilder_v2()
    builder.build()
