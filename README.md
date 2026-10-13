# Arduino 双轮 PID 差速控制与可视化调参

本项目面向两轮差速底盘，使用 Arduino、TB6612 和两路增量式编码器完成左右轮独立速度闭环控制。它提供两种控制入口：直接下发左右轮 RPM，或下发线速度/角速度并由固件转换为左右轮目标转速。固件同时输出轮速、PWM、误差和轮式里程计，PC 上位机可实时调参与保存数据。

## 能力边界

- 左右电机各自独立 PID 速度闭环，支持分别或同时修改参数。
- 差速运动学：`CMDVEL v w` 转换为左右轮 RPM。
- 编码器测速、左右轮累计脉冲与二维轮式里程计 `(x, y, theta)`。
- 串口通信超时自动刹车；上位机运动指令每 200 ms 自动续发。
- 上位机显示左/右轮目标和实际转速共四条曲线，支持仿真与 CSV 导出。

这不是完整 SLAM 或 ROS 导航工程；本目录只负责底层双轮闭环控制和上位机联调接口。

## 目录结构

```text
PID/
  arduino_motor_pid/
    arduino_motor_pid.ino  # 程序入口与串口循环
    RobotController.*      # 双轮控制、串口协议、差速运动学
    MotorDriver.*          # TB6612 单通道驱动
    Encoder.*              # 两路编码器中断计数
    PidController.h        # PID 与积分限幅
    Odometry.*             # 差速轮式里程计
  pc_tuner/
    pid_tuner.py           # Tkinter 调参界面
    transport.py           # 串口连接与双轮仿真模型
    telemetry.py           # 遥测数据模型与解析
    plot_canvas.py         # 左右轮曲线绘制
  requirements.txt
```

## 默认接线

默认适用于 Arduino Uno/Nano。电机供电、Arduino 和 TB6612 的 GND 必须共地。

| 功能 | TB6612/编码器端 | Arduino |
| --- | --- | --- |
| 左轮 PWM | PWMA | D9 |
| 左轮方向 | AIN1 / AIN2 | D7 / D8 |
| 右轮 PWM | PWMB | D10 |
| 右轮方向 | BIN1 / BIN2 | D5 / D6 |
| 驱动使能 | STBY | D4 |
| 左编码器 A 相 | 外部中断 | D2 |
| 左编码器 B 相 | 方向判断 | D11 |
| 右编码器 A 相 | 外部中断 | D3 |
| 右编码器 B 相 | 方向判断 | D12 |
| TB6612 VCC | 逻辑电源 | 5V |
| TB6612 VM | 电机电源 | 外部电机电源正极 |

若左右轮实际方向与指令相反，修改 `RobotController.cpp` 顶部的 `LEFT_MOTOR_INVERT` 或 `RIGHT_MOTOR_INVERT`。编码器计数方向不对时，修改对应的 `*_ENCODER_INVERT`。

## 上传和运行

用 Arduino IDE 打开 `arduino_motor_pid/arduino_motor_pid.ino`，选择正确的板卡和串口后上传。默认串口波特率为 `115200`。

安装上位机依赖并运行：

```powershell
cd D:\vs\zxm\PID
pip install -r requirements.txt
python pc_tuner\pid_tuner.py
```

未接硬件时，点击 `Sim` 可验证双轮曲线、指令和数据导出流程。

## 串口协议

每行一条 ASCII 命令，以换行结束。

```text
# 左右轮直接给定 RPM
WHEEL 80 80
WHEEL -60 60

# 差速速度指令：线速度 m/s，角速度 rad/s
CMDVEL 0.12 0
CMDVEL 0 1.2

# PID：同时设置、仅左轮、仅右轮
PID SPEED 1.2 0.35 0.01
PID LEFT 1.3 0.40 0.01
PID RIGHT 1.1 0.32 0.01

# 车辆标定参数
CONFIG CPR 1320
CONFIG WHEEL_DIAM 0.065
CONFIG WHEEL_BASE 0.155
CONFIG MAXRPM 220
CONFIG REPORT 50
CONFIG TIMEOUT 600

ZERO
STOP
STATUS
```

参数说明：

- `CPR`：控制程序实际计入的每个车轮输出轴一圈脉冲数。当前 A 相双沿计数，数值必须按实际编码器测量或计算。
- `WHEEL_DIAM`：驱动轮直径，单位 m。
- `WHEEL_BASE`：左右轮接地点中心距离，单位 m。
- `TIMEOUT`：超过这个时间没有新的 `WHEEL` 或 `CMDVEL` 指令，固件自动刹车，单位 ms。

固件的遥测行格式如下，可直接供树莓派端程序或 ROS 串口桥解析：

```text
DATA,t=1234,mode=CMDVEL,target_l=35.27,rpm_l=34.80,pwm_l=106.2,err_l=0.47,target_r=35.27,rpm_r=35.10,pwm_r=108.1,err_r=0.17,x=0.1420,y=0.0003,theta=0.0040,v=0.119,w=0.002
```

## 标定顺序

1. 悬空测试 `WHEEL 40 40`，确认两个轮子均向前；必要时调整电机反向配置。
2. 检查两个轮子的 RPM 均为正；必要时调整编码器反向配置。
3. 实测一圈脉冲数，设置 `CPR`；测量轮径和轮距，设置 `WHEEL_DIAM` 与 `WHEEL_BASE`。
4. 分别对左、右轮做 `WHEEL 60 0`、`WHEEL 0 60` 阶跃实验，再用 `PID LEFT`、`PID RIGHT` 微调。
5. 最后测试 `CMDVEL 0.12 0` 直行和 `CMDVEL 0 1.0` 原地转向，检查轮速与里程计趋势。
