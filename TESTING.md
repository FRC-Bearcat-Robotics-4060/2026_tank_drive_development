# Simulation test procedure + log analysis

At startup the code calls `DataLogManager.start()`, which writes every NetworkTables value, the console output, the Driver Station state and the joystick inputs to a `.wpilog` file:

- Simulation: the project's **`logs/`** folder (already in `.gitignore`, never committed)
- Real robot: a USB stick if one is plugged into the roboRIO, otherwise `/home/lvuser/logs` (a roboRIO 1 only has about 64 MB free, so **plug in a USB stick on the real robot**)

## 0. Set up the controller

Pick one:

- **Xbox controller**: plug it into the computer and drag it from System Joysticks onto Joystick[0]. Left stick = left side, right stick = right side.
- **Keyboard**: DS → Keyboard 0 Settings, set the axis count to **6** and assign keys to **Axis 5**: **decrease (−) = I, increase (+) = K**.
  (Same as Axis 1: W is the "decrease" key → axis value −1 → negated by the code = forward.) Drag Keyboard 0 onto Joystick[0].

## 1. Run the test

```powershell
cd "C:\Users\hongy\Downloads\frc tank pro 2\frc tank pro 2\pro 2 tank - tank-drive"
.\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```

Go through the steps in order. **Hold each input for 3 seconds, then release and wait 2 seconds** (the analysis script splits maneuvers on steady inputs lasting ≥ 0.8 s):

| # | Action (keyboard / controller) | Expected |
|---|---|---|
| 0 | Stay **Disabled** for 2 s, then select **Teleoperated** | — |
| 1 | **W + I** / both sticks forward | Drives straight forward |
| 2 | **S + K** / both sticks back | Drives straight backward |
| 3 | **W only** / left stick only | Turns right in place (clockwise) |
| 4 | **I only** / right stick only | Turns left in place (counter-clockwise) |
| 5 | **W + K** / left forward, right back | Spins quickly clockwise |
| 6 | (controller only) both sticks **half way** | Drives straight slowly (tests the squaring curve) |
| 7 | Select **Disabled**, wait 2 s, then **close the simulation window** | Log is finished |

## 2. Analyze

When you are done, tell the mentor; the log is the newest `.wpilog` in `logs/`.

To run the analysis yourself:

```powershell
pip install robotpy-wpiutil numpy matplotlib
python tools\analyze_drive_log.py logs
```

It creates `<log name>_report/` next to the log, containing `report.md`, `timeseries.png` and `path.png`.

### What is checked automatically

| Check | Spec it verifies |
|---|---|
| Followers equal leaders (PWM8 = PWM3, PWM5 = PWM4) | Bug 1 fixed |
| \|PWM\| ≤ 0.5 | `kMaxOutput` |
| Stick → output = deadband 0.05 + squaring + × 0.5 | `kDeadband`, `kSquareInputs`, `kMaxOutput` |
| Right stick Axis 5 is present | Bug 2 fixed, and the controller/keyboard is set up correctly |
| All outputs are 0 while disabled | Safety |
| Right side PWM reversed, left side not | `kRightInverted` / `kLeftInverted` |
| Per maneuver: wheel speed, t63 / t90, yaw rate, turn direction | Physical behavior (kitbot preset parameters) |
| Minimum battery voltage, peak current | Battery-sag simulation |

You can also drag the `.wpilog` straight into **AdvantageScope** to view the curves and the `/Drive/SimPose` 2D field path.
