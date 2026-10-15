# Branch `tank-drive` — 底盘开发 + 仿真

基于 `sim-field`。把 tank drive 底盘整理好、修掉两个 bug，并接上场地仿真。

## 结构

| 文件 | 内容 |
|---|---|
| `Constants.java` | 所有数字：PWM 通道、反转、最大输出、死区、手柄轴号、仿真起点 |
| `Drivetrain.java` | 4 个电机、跟随关系、`DifferentialDrive`、Field2d、底盘物理仿真、电池压降仿真 |
| `Robot.java` | 只负责把手柄接到 `Drivetrain`：左摇杆 Y → 左侧，右摇杆 Y → 右侧 |

## 修掉的 bug

1. **从动电机不转** → `m_leftLeader.addFollower(m_leftFollower)` / `m_rightLeader.addFollower(m_rightFollower)`。
   反转只设在主电机上：`set()` 会先应用主电机的反转，再把结果传给从动电机（已从 WPILib 2026 字节码确认）。
2. **左右读同一个轴** → 左侧读 `kLeftYAxis`(1)，右侧读 `kRightYAxis`(5)。

## 其他改动

- `kMaxOutput = 0.5`：**第一次上真机限制半速**，方向确认没问题后改成 1.0。
- `kDeadband = 0.05`：摇杆回中不归零时机器人不会自己慢慢动。
- `kSquareInputs = true`：低速更细腻，最高速不变。
- `disabledInit()` 里主动停电机。
- 仪表盘：`Drive/Left Output`、`Drive/Right Output`（只读）、`Field`。
  **故意没有**用 `putData` 发布 `DifferentialDrive` 组件：它是可写的执行器，仪表盘能直接写电机速度，会覆盖 tankDrive 的输出（真机上也是安全隐患）。
- 仿真里加了**电池压降**：猛加速时电流大，`RobotController.getBatteryVoltage()` 会掉。

## 怎么跑仿真

在 `pro 2 tank - tank-drive` 文件夹里：

```powershell
.\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```

Sim GUI 里：

1. **NetworkTables → SmartDashboard → Field** 打开场地窗口。
2. Robot State 选 **Teleoperated**。
3. 手柄（二选一）：
   - **推荐：插一个 Xbox 手柄到电脑**，它会出现在 System Joysticks 里，拖到 Joystick[0]。左摇杆=左侧、右摇杆=右侧，跟真机一样。
   - **只用键盘**：tank drive 需要两个 Y 轴（轴 1 和轴 5），Keyboard 0 默认只配了前几个轴。打开 **DS → Keyboard 0 Settings**，把轴数加到 6，给 **Axis 5** 设两个键（比如 I / K），再把 Keyboard 0 拖到 Joystick[0]。这样 W/S 控左侧、I/K 控右侧。

## 验证清单（仿真里逐条过）

| 操作 | PWM 3 / 8（左） | PWM 4 / 5（右） | 场地上 |
|---|---|---|---|
| 两边都推前 | 3 与 8 相同，> 0 | 4 与 5 相同，< 0（右侧反转） | 直线往前 |
| 只推左侧 | 有值 | 0 | 原地右转（顺时针） |
| 只推右侧 | 0 | 有值 | 原地左转（逆时针） |
| 左前右后 | > 0 | > 0 | 原地快速顺时针转 |
| 切到 Disabled | 全 0 | 全 0 | 停下 |

最大值应该是 **±0.5**（`kMaxOutput`），不是 ±1。

## 上真机前（现场）

1. **架空**，有人守空格键。
2. 在 DS 的 USB 页确认右摇杆 Y 轴确实是 **Axis 5**，不是就改 `kRightYAxis`。
3. 推左摇杆 → 左侧两个轮子都往前？推右摇杆 → 右侧两个轮子都往前？
   - 某一侧反了 → 改 `kLeftInverted` / `kRightInverted`。
   - 同一侧两个轮子方向相反（互相顶）→ 那个从动电机的安装方向和主电机相反，需要单独处理，先停下来报告。
4. 都对了，再落地慢慢开；之后把 `kMaxOutput` 调到 1.0。

## 已知限制

- 仿真底盘参数是 **KOP kitbot 预设**（每侧 2×CIM、10.71:1、6 寸轮），不是队里真实参数。
- 没有编码器/陀螺仪，所以没有里程计；场地上显示的是物理模型算的**真值**。
- 场地没有墙壁碰撞。

## 下一步

- 填入真实的底盘参数（电机型号、减速比、轮径、质量、轮距）。
- 加编码器 + 陀螺仪 → `DifferentialDriveOdometry`，场地上同时画"真值"和"估计"。
- 改成 Command-based（`Drivetrain` 变成 `SubsystemBase`，手柄绑定移到 `RobotContainer`）。
