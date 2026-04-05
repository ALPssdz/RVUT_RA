import adi
import numpy as np
import time

class RF_Stage2_Dwell:
    """
    Cognitive RF Tier 2: Vectorized Dwell Phase and Vision Object Processing.
    使用极致的 GPU 级 Numpy 向量化矩阵操作替代所有 Python 慢速获取循环！
    彻底防范由于 CPU 算力跟不上 USB 传输而产生的 DMA Overflow (满屏横向杂音断带)。
    """
    def __init__(self, sdr_instance):
        self.sdr = sdr_instance
        self.target_width = 640
        self.target_height = 640
        
        # 4096 点的高分辨率 FFT 窗口
        self.fft_size = 4096
        self.window = np.blackman(self.fft_size)
        
        # 严格遵守 YOLO 数据集训练色彩空间
        self.vmin = -60
        self.vmax = 30
        
    def generate_waterfall_tensor(self, center_freq):
        """
        以矩阵填充迭代产生出带有预标记属性的 Numpy 640x640 类型数据流列结构数组，
        无损传唤 NPU/YOLO 相关张量计算端点进行推理。
        """
        self.sdr.rx_lo = int(center_freq)
        
        # 强制冲刷掉上游切频残留下来的老旧陈腐缓存
        try:
            _ = self.sdr.rx()
            _ = self.sdr.rx()
        except:
            pass
            
        # ==========================================================
        # 【终极快门：65毫秒宏观时空一口气吞噬】
        # 这里仅调用一次 rx() 返回 2,621,440 个样本。全量霸道截取！
        # 这个动作由底层硬件 DMA 控制器瞬间完成，没有任何一皮秒的丢失缝隙。
        # 彻底解决屏幕上的全段位横向条纹撕裂伪影。
        # ==========================================================
        time_rx = time.time()
        raw_iq = self.sdr.rx()
        
        # 原封不动保存这块绝对完美、相位相连的玉璞，交给后续 S3
        self.last_buffer_iq = raw_iq
        
        # [极为关键的防爆盾] 拔除硬件本振直流泄漏！
        normalized_iq = raw_iq / 32768.0
        normalized_iq = normalized_iq - np.mean(normalized_iq)
        
        # 裁剪掉多余的微小尾巴以对准 640 x 4096 结构
        valid_length = self.target_height * self.fft_size
        if len(normalized_iq) > valid_length:
            normalized_iq = normalized_iq[:valid_length]
        elif len(normalized_iq) < valid_length:
            # 补齐防崩
            normalized_iq = np.pad(normalized_iq, (0, valid_length - len(normalized_iq)))
            
        # Numpy 黑魔法：直接变成二维矩阵，并行计算所有 640 行的 FFT！这比 Python 的 For 循环快几十倍。
        reshaped = normalized_iq.reshape((self.target_height, self.fft_size))
        windowed = reshaped * self.window
        
        # 向量化傅立叶变换
        fft_data = np.fft.fftshift(np.fft.fft(windowed, axis=1), axes=1)
        power_db = 20 * np.log10(np.abs(fft_data) + 1e-12)
        
        # 在频域上横向压缩！4096 -> 640
        pool_size = self.fft_size // self.target_width
        trim_side = (self.fft_size - (pool_size * self.target_width)) // 2
        
        trimmed_db = power_db[:, trim_side : -trim_side]
        pool_reshaped = trimmed_db.reshape((self.target_height, self.target_width, pool_size))
        
        # 用并行池化算法把细致的频点压缩出最高强度的轮廓
        waterfall = np.max(pool_reshaped, axis=2)
        
        # 运用 OpenCV 色彩算子映射平面为仿生类红外三通道矩阵
        waterfall_clipped = np.clip(waterfall, self.vmin, self.vmax)
        waterfall_norm = ((waterfall_clipped - self.vmin) / (self.vmax - self.vmin) * 255.0)
        waterfall_uint8 = waterfall_norm.astype(np.uint8)
        
        import cv2
        waterfall_bgr = cv2.applyColorMap(waterfall_uint8, cv2.COLORMAP_HOT)
        
        return waterfall_bgr
