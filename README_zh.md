# RF-RA8P1-UAV-Tracker

[English](README.md) | **简体中文**

基于 **RA8P1 主控裁决** 与 **RK3588 外置射频算力单元** 的 5.8 GHz 无人机射频预警系统。

本工程已从早期 RF + K230 视觉融合方案调整为 RA8P1 主控架构：K230 视觉节点已移除，RA8P1 作为系统主控核心和最终裁决端，香橙派 5/RK3588 作为外置算力与 HDMI 显示终端，负责射频检测算法、数据库和可视化展示。

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
  - PyQt HDMI 大屏显示终端
  - 通过 JDBG 虚拟串口向 RA8P1 上报检测结果

ZYNQ-7020 + AD9364
  - 5.8 GHz IQ 采集前端
  - libiio / pyadi-iio 网络控制
```

## 通信链路

CPKHMI-RA8P1 当前实测链路为板载 J-Link OB 虚拟串口，RA8P1 侧使用 SCI9 UART：

```text
设备名：/dev/ttyACM0
波特率：2000000
串口格式：8-N-1
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

正式模式控制约束：

```text
RA8P1_REQUIRED=True
RA8P1 未在线：RF 采集管道锁定
本地 GUI 启动按钮：不能绕过 RA8P1 主控直接启动采集
RA8P1 START_SCAN：唯一正式采集放行命令
RA8P1 STOP_SCAN / RESET_ALERT / RUN_CALIBRATION：香橙派端按主控命令执行
```

## 射频检测流水线

```text
IQ Samples
  -> S1 Kurtosis-Weighted RSSI Pre-scan
  -> S2 Waterfall Display
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
│   ├── config.py                    # SDR + RA8P1 JDBG UART 配置
│   └── main_rf_pipeline.py          # S1/S2/S3 RF 检测主控
├── rf_zynq/
│   ├── rf_stage1_rssi_scan.py
│   ├── rf_stage2_waterfall.py
│   ├── rf_stage3_cyclostationary.py
│   ├── calibrate_s3.py
│   └── yolo/                         # 历史训练资料，正式检测链路不加载
├── protocol/
│   ├── link_protocol.py             # RA8P1 <-> RK3588 消息编解码
│   ├── ra8p1_link.py                # JDBG 虚拟串口链路
│   └── messages.md
├── mock/
│   └── mock_ra8p1.py                # RA8P1 主控 mock
├── ui_qt/
│   └── gui_host.py                  # HDMI 大屏显示终端
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

## 误报分析数据

正式运行时会自动记录诊断 session：

```text
diagnostics/captures/session_YYYYmmdd_HHMMSS/
├── events.jsonl      # 候选、S3通过、RA8P1裁决事件元数据
├── runtime.log       # GUI/Hub完整运行日志
├── frames/           # 对应瀑布图 JPG
└── iq/               # 关键告警事件 IQ 压缩包
```

香橙派现场打包：

```bash
cd ~/RVUT_RA/RF-Vision-UAV-Tracker
python3 tools/package_diagnostics.py
```

笔记本拉回分析：

```bash
scp orangepi@192.168.31.34:~/RVUT_RA/RF-Vision-UAV-Tracker/diagnostics/*.tar.gz .
```

当前误报抑制策略：

```text
S3 SDS 门限：1.08
普通检出：PSR / CFS / AFS 三项硬验证必须全部通过
强 NCC 旁路：仍要求 PSR 通过，并且 CFS/AFS 至少一项通过
诊断写盘：后台异步保存，避免阻塞心跳
RA8P1 心跳超时：10000 ms
```

## 当前迁移状态

- K230 运行链路已移除
- PyQt 中 K230 显示框已替换为 RA8P1 主控裁决面板
- RA8P1 JDBG UART 协议与 RK3588 链路客户端已加入
- Hub 已接收真实 RA8P1 `MASTER_DECISION` 回包并用于裁决显示
- 比赛默认 `RA8P1_REQUIRED=True`：RA8P1 未接入时采集管道锁定
- 开发调试时可手动将 `RA8P1_REQUIRED=False`，临时启用 RF 本地降级路径
