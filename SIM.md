# Branch `sim-field` — Field simulation

Draws the robot on a field in simulation and lets you drive it with a controller or the keyboard. **The driving code is unchanged**, so the simulation shows exactly what the real robot would do with this code, including its two known bugs.

## What changed

Only `src/main/java/frc/robot/Robot.java`:

| Added | Purpose |
|---|---|
| `Field2d m_field` | Field widget, published to `SmartDashboard/Field` |
| `DifferentialDrivetrainSim m_driveSim` | Drivetrain physics model (created only in simulation) |
| `simulationInit()` | Builds the model and places the robot at the start pose `(2.0 m, 4.0 m)` |
| `simulationPeriodic()` | Every 20 ms: read the two leader motor commands → convert to volts → step the physics → update the pose on the field |

## How to run

In this folder (`pro 2 tank - sim-field`):

```powershell
.\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```

(In WPILib VS Code: `WPILib: Simulate Robot Code` → check Sim GUI only. **Do not click Run above `main()`.**)

In the Sim GUI:

1. Menu **NetworkTables → SmartDashboard → Field** to open the field window.
2. **Right-click** in the field window and pick a built-in season field image from its settings (optional; without it the background is a blank grid).
3. Drag **Keyboard 0** onto **Joystick[0]** and set Robot State to **Teleoperated**.
4. Click the window to give it focus, then hold **W / S**.

## What you should see

- W drives forward along the current heading, S drives backward.
- A / D: **the robot does not turn** — this is bug 2 (both sides read the same axis).
- To return to the start pose, restart the simulation.

## Assumptions and limitations

- **Drivetrain parameters are placeholders**: the KOP kitbot preset (2 × CIM per side, 10.71:1, 6 in wheels). Replace them with the team's real motor type, gear ratio, wheel size, mass and track width before trusting speeds or accelerations.
- The model assumes both motors on each side produce torque, but the robot code currently **only drives the leaders** (bug 1), so the real robot has about half the pushing force.
- The field shows the **ground-truth pose** computed by the physics model. There is no odometry or encoder yet; once odometry is added, draw both the "true" and the "estimated" robot on the field.
- There are no field walls; the robot can drive off the field.

## Next steps (for students)

1. Fix bug 1 (`addFollower`) and bug 2 (turning), then confirm in simulation that the robot turns.
2. Move the drivetrain parameters into `Constants` and replace them with the real values.
3. Add encoders + a gyro + `DifferentialDriveOdometry`, and draw a second robot (the estimated pose) on the field to compare against the ground truth.
