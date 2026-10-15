# Branch `sim-field` — 场地仿真

在仿真里把机器人画在场地上，用手柄/键盘开着跑。**驾驶代码没改**，所以仿真里看到的就是真机用这份代码会出现的行为（包括两个已知 bug）。

## 改了什么

只改了 `src/main/java/frc/robot/Robot.java`：

| 新增 | 作用 |
|---|---|
| `Field2d m_field` | 场地组件，发布到 `SmartDashboard/Field` |
| `DifferentialDrivetrainSim m_driveSim` | 底盘物理模型（只在仿真里创建） |
| `simulationInit()` | 建模型、把机器人放到起点 `(2.0 m, 4.0 m)` |
| `simulationPeriodic()` | 每 20 ms：读两个主电机的指令 → 换算成电压 → 推进物理 → 更新场地上的位置 |

## 怎么跑

在这个文件夹（`pro 2 tank - sim-field`）里：

```powershell
.\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```

（用 WPILib VS Code 的话：`WPILib: Simulate Robot Code` → 只勾 Sim GUI。**不要点 main() 上的 Run。**）

Sim GUI 里：

1. 菜单 **NetworkTables → SmartDashboard → Field**，打开场地窗口。
2. 在场地窗口里**右键**，从设置里选一张内置的赛季场地图片（不选也能用，只是背景是空白网格）。
3. **Keyboard 0** 拖到 **Joystick[0]**，Robot State 选 **Teleoperated**。
4. 点一下窗口获得焦点，按住 **W / S**。

## 预期看到

- W：机器人沿当前朝向往前开，S：往后。
- A / D：**不会转弯** —— 这就是 bug 2（左右都读同一个轴）。
- 想让它回到起点：重启仿真。

## 已知限制 / 假设

- **底盘参数是假设的**：KOP kitbot 预设（每侧 2×CIM、10.71:1、6 寸轮）。要换成队里真实的电机型号、减速比、轮径、质量、轮距，速度和加速度才可信。
- 模型假设每侧两个电机都出力，但真机代码目前**只驱动主电机**（bug 1），真机实际推力只有一半。
- 显示的是**真值位姿**（物理模型算出来的），还没有里程计/编码器。以后加里程计时，场地上应同时画"真值"和"估计值"两个机器人。
- 场地没有墙壁碰撞，机器人可以开出场地。

## 下一步（给学生）

1. 修 bug 1（`addFollower`）和 bug 2（转向），在仿真里确认能转弯。
2. 把底盘参数抽到 `Constants`，换成真实值。
3. 加编码器 + 陀螺仪 + `DifferentialDriveOdometry`，场地上画第二个机器人（估计位姿），对比真值。
