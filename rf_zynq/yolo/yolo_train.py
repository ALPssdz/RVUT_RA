"""
YOLO Training Script v2.0 - Maximum Precision Configuration
=============================================================
使用物理对齐数据集 v2.0 进行高精度重训。
模型：YOLOv8s (small) - 在精度与实时推理速度之间最优平衡。
"""
from ultralytics import YOLO
import os

def train_rf_model():
    model = YOLO('e:/Myprojects/RF-Vision-UAV-Tracker/rf_zynq/yolo/yolov8s.pt')

    yaml_config = "e:/Myprojects/RF-Vision-UAV-Tracker/rf_yolo_dataset/rf_uav.yaml"
    project_dir = "e:/Myprojects/RF-Vision-UAV-Tracker/rf_zynq/yolo/runs"

    print("[Train] 开始在物理对齐数据集 v2.0 上进行高精度训练...")
    results = model.train(
        data       = yaml_config,
        epochs     = 300,          # 充分收敛
        imgsz      = 640,
        batch      = 8,            # 保守 batch，兼容 4GB 以下显存
        device     = 0,            # GPU 0
        workers    = 0,            # Windows 兼容性
        cos_lr     = True,         # 余弦退火学习率，防后期震荡
        patience   = 50,           # Early Stopping
        project    = project_dir,
        name       = "rf_uav_v2_aligned",
        exist_ok   = True,
        # 数据增强：关闭 mosaic 防止频谱图被切碎破坏物理特征
        mosaic     = 0.0,
        # 开启翻转增强（频谱沿时间轴可以上下翻，不影响物理意义）
        flipud     = 0.5,
        fliplr     = 0.0,          # 频域不可左右翻（对称性不成立）
        # 轻微的颜色抖动（仿真不同增益设置下的色彩偏差）
        hsv_h      = 0.0,
        hsv_s      = 0.2,
        hsv_v      = 0.2,
    )

    best = os.path.join(project_dir, "rf_uav_v2_aligned", "weights", "best.pt")
    print(f"\n[Train] 训练完成！最佳权重已落地: {best}")

if __name__ == '__main__':
    train_rf_model()
