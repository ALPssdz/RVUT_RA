# RF-RA8P1-UAV-Tracker

[English](README.md) | **简体中文**

基于 **RA8P1 主控裁决** 与 **RK3588 射频算法协处理** 的 5.8 GHz 无人机射频预警系统。

本工程已从早期 RF + K230 视觉融合方案调整为 RF-only 架构：K230 视觉节点已移除，RA8P1 作为系统主控核心，香橙派 5/RK3588 负责射频检测算法、数据库和 HDMI 大屏上位机。

## 系统角色

```text
RA8P1 主控制器
  - 系统状态机
  - 启停/标定控制
  - 接收 RF 检测报告
  - 最终告警裁决
  - LED/蜂鸣器/比赛外设控制

香橙派 5 / RK3588
  - RF Agent
  - S1/S2/S3 射频检测流水线
  - SQLite 告警数据库
  - PyQt HDMI 大屏上位机
  - 通过 UART 向 RA8P1 上报检测结果

ZYNQ-7020 + AD9364
  - 5.8 GHz IQ 采集前端
  - libiio / pyadi-iio 网络控制
```

## 通信链路

RA8P1 与香橙派使用 UART：

```text
波特率：921600
格式：8N1
初期协议：JSON Lines + checksum
```

RA8P1 下发：

```text
START_SCAN
STOP_SCAN
RUN_CALIBRATION
RESET_ALERT
GET_STATUS
```

RK3588 上报：

```text
HEARTBEAT
AGENT_READY
CALIBRATION_DONE
DETECTION_REPORT
FAULT_REPORT
```

RA8P1 返回：

```text
MASTER_DECISION
```

## 射频检测流水线

```text
IQ Samples
  -> S1 Kurtosis-Weighted RSSI Pre-scan
  -> S2 Waterfall Generation / YOLO Assist
  -> S3 Cyclostationary Audit
  -> RF Detection Report
  -> RA8P1 Master Decision
  -> SQLite + HDMI PyQt Console
```

## 软件结构

```text
RF-Vision-UAV-Tracker/
├── system_hub.py                    # 当前 PyQt + RF Agent 入口
├── backend_rk3588/
│   ├── config.py                    # SDR + RA8P1 UART 配置
│   └── main_rf_pipeline.py          # S1/S2/S3 RF 检测主控
├── rf_zynq/
│   ├── rf_stage1_rssi_scan.py
│   ├── rf_stage2_waterfall_yolo.py
│   ├── rf_stage3_cyclostationary.py
│   ├── calibrate_s3.py
│   └── rknn_infer.py
├── protocol/
│   ├── uart_protocol.py             # RA8P1 <-> RK3588 消息编解码
│   └── messages.md
├── mock/
│   └── mock_ra8p1.py                # RA8P1 主控 mock
├── ui_qt/
│   └── gui_host.py                  # HDMI 大屏上位机
├── database/
│   └── db_manager.py
├── tools/
└── deploy_orangepi.sh
```

## 启动

```bash
python3 system_hub.py
```

单独执行 S3 背景标定：

```bash
python3 rf_zynq/calibrate_s3.py
```

协议 mock 示例：

```bash
python3 mock/mock_ra8p1.py
```

## 当前迁移状态

- K230 运行链路已移除
- PyQt 中 K230 显示框已替换为 RA8P1 主控裁决面板
- RA8P1 UART 协议骨架已加入
- 当前 RF 告警仍保留本地过渡确认路径
- 下一阶段将把最终告警确认迁移到真实 RA8P1 UART 回包

