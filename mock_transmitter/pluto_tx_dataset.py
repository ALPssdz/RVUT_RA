import os
import time
import numpy as np
from scipy.signal import resample_poly
import adi

def start_transmission():
    print("============================================================")
    print("RF-Vision 靶机模拟器: 数据集真实电磁波物理重放 (Dataset Replay)")
    print("============================================================")
    
    dataset_path = r"e:\Myprojects\RF-Vision-UAV-Tracker\Drone RF Data\DJI MINI4 PRO\VTSBW=20\pack2_0-1s.iq"
    
    if not os.path.exists(dataset_path):
        print(f"[!] 致命异常：找不到数据集基底文件，路径检查失败！\n-> {dataset_path}")
        return
        
    # 【底层物理学警告】：我们绝对不能尝试在 Python 层做流式发送！
    # 只要发送端一断流，接收端的 AGC 就会瞬间把背景底噪放大 1000 倍！这就是产生满屏横向条纹（马赛克）的元凶。
    #
    # 从 28ms 的地方切出来，大疆爆发的最精髓中心
    offset_bytes = 2800000 * 8 
    
    # 【重大修正】：将切割送入靶机的缓冲从 250万（20MB）大幅缩水至 40万（3MB）！
    # 因为 PlutoSDR 的 FPGA (TX Cyclic Buffer) 物理内存上限极小，强行灌入 250万 会导致驱动静默崩溃，进而雷达什么也听不到。
    frames_to_read = 400000 
    
    print("[1] 正在加载物理高能片段阵体...")
    raw_iq_100m = np.fromfile(dataset_path, dtype=np.complex64, count=frames_to_read, offset=offset_bytes)
    
    print("[2] 正在利用滤波器组进行高保真重采样 (降速防时光畸变) : 100MSPS -> 40MSPS...")
    # 抽取降采样：100M 变 40M 就是 乘以 2，除以 5
    raw_iq_40m = resample_poly(raw_iq_100m, 2, 5)
    
    duration_ms = len(raw_iq_40m) / 40e6 * 1000
    print(f"    时域重对齐完毕，有效驻留切片长: {len(raw_iq_40m)} 节点 (真实空中时长: {duration_ms:.1f} 毫秒).")

    # PlutoSDR 发射器强制吸收 [-32768, 32767] 大数字定点，必须做放大投射
    # PlutoSDR 发射器强制吸收 [-32768, 32767] 大数字定点，必须做放大投射
    max_amp = np.max(np.abs(raw_iq_40m))
    if max_amp == 0: max_amp = 1.0 # 避免除零
    
    # 【解除火力封锁】：直接拉满至 32700 贴脸极限输出 (避开极细微的 32767 硬件溢出即可)
    target_scale = 32700.0 
    normalized_iq = (raw_iq_40m / max_amp) * target_scale

    uri = "ip:192.168.31.20"
    print(f"\n[3] 正在挂载网络物理执行端点 {uri} ...")
    try:
        sdr = adi.Pluto(uri)
    except Exception as e:
        print(f"[!] SDR 连接超时崩溃: {e}")
        return
        
    sdr.sample_rate = int(40e6)
    sdr.tx_rf_bandwidth = int(40e6)
    sdr.tx_lo = int(2420e6)          # 恢复原始物理特征截获频点 (2.45 GHz)
    
    # 【硬件底大缸解封】：把 PlutoSDR 的射频放大器（Tx Gain）直接全功率 0dB 怒吼！
    sdr.tx_hardwaregain_chan0 = 0 
    
    # ★ 使用 FPGA 级硬件内存闭环，不断重复这段图传录音
    sdr.tx_cyclic_buffer = True
    
    print(f"\n[!] 物理信令发射准备就绪。靶机正以 {sdr.sample_rate/1e6}MSPS 发射真实的 DJI 频段！")
    print(f"    射频驻留点锁定: {sdr.tx_lo/1e6} MHz")
    print(">>> 正在不间断辐射图传载波，您随时可以按 Ctrl+C 安全阻断并重置靶机。")
    
    # PUSH ！！由 FPGA 接管一切！
    sdr.tx(normalized_iq)
    
    try:
        while True:
            time.sleep(1)
                
    except KeyboardInterrupt:
        print("\n[!] 用户硬终端接收，正在抹除 FPGA 缓冲死区...")
        sdr.tx_cyclic_buffer = False
        sdr.tx(np.zeros(1024))
        try:
            sdr.tx_destroy_buffer()
        except:
            pass
        print("[!] 射频辐射已安全阻断，靶机已关机。")

if __name__ == "__main__":
    start_transmission()
