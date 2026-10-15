# 仿真测试流程 + 日志分析

代码启动时会调用 `DataLogManager.start()`，把所有 NetworkTables 数据、控制台输出、DS 状态和手柄输入写进一个 `.wpilog` 文件：

- 仿真：项目里的 **`logs/`** 文件夹（已在 `.gitignore` 里，不会被提交）
- 真机：插了 U 盘就写 U 盘，否则写 roboRIO 的 `/home/lvuser/logs`（roboRIO 1 只剩约 64 MB，**真机上建议插 U 盘**）

## 0. 准备手柄

二选一：

- **Xbox 手柄**：插电脑，在 System Joysticks 里拖到 Joystick[0]。左摇杆 = 左侧，右摇杆 = 右侧。
- **键盘**：DS → Keyboard 0 Settings，轴数改成 **6**，给 **Axis 5** 设键：**减（−）= I，增（+）= K**。
  （和 Axis 1 一样：W 是"减"键 → 轴值 −1 → 代码取反后 = 前进。）Keyboard 0 拖到 Joystick[0]。

## 1. 跑测试

```powershell
cd "C:\Users\hongy\Downloads\frc tank pro 2\frc tank pro 2\pro 2 tank - tank-drive"
.\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```

按顺序做，**每一步按住 3 秒，松开等 2 秒**（分析脚本靠"稳定输入 ≥ 0.8 s"来切分动作）：

| # | 操作（键盘 / 手柄） | 预期 |
|---|---|---|
| 0 | 保持 **Disabled** 2 秒，再选 **Teleoperated** | — |
| 1 | **W + I** / 两个摇杆都推前 | 直线前进 |
| 2 | **S + K** / 两个摇杆都拉后 | 直线后退 |
| 3 | **只按 W** / 只推左摇杆 | 原地右转（顺时针） |
| 4 | **只按 I** / 只推右摇杆 | 原地左转（逆时针） |
| 5 | **W + K** / 左推右拉 | 原地快速右转 |
| 6 | （仅手柄）两个摇杆**推一半** | 直线慢速（测平方曲线） |
| 7 | 选 **Disabled**，等 2 秒，**关掉仿真窗口** | 日志写完 |

## 2. 分析

跑完告诉 mentor 一声，日志在 `logs/` 里最新的 `.wpilog`。

自己跑分析：

```powershell
pip install robotpy-wpiutil numpy matplotlib
python tools\analyze_drive_log.py logs
```

会在日志旁边生成 `<日志名>_report/`：`report.md`、`timeseries.png`、`path.png`。

### 自动检查的项目

| 检查 | 对应规格 |
|---|---|
| 从动 = 主电机（PWM8 = PWM3，PWM5 = PWM4） | bug 1 已修 |
| \|PWM\| ≤ 0.5 | `kMaxOutput` |
| 摇杆 → 输出 = 死区 0.05 + 平方 + ×0.5 | `kDeadband`、`kSquareInputs`、`kMaxOutput` |
| 右摇杆 Axis 5 存在 | bug 2 已修，且手柄/键盘配置正确 |
| Disabled 时全部为 0 | 安全 |
| 右侧 PWM 反向、左侧不反向 | `kRightInverted` / `kLeftInverted` |
| 每个动作：轮速、t63 / t90、转向角速度、转向方向 | 物理表现（kitbot 预设参数） |
| 最低电池电压、峰值电流 | 电池压降仿真 |

也可以直接把 `.wpilog` 拖进 **AdvantageScope** 看曲线和 `/Drive/SimPose` 的 2D 场地轨迹。
