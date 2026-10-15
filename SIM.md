# Branch `tank-drive` — Drivetrain development + simulation

Based on `sim-field`. Cleans up the tank-drive drivetrain, fixes both bugs, and keeps the field simulation.

## Structure

| File | Contents |
|---|---|
| `Constants.java` | Every number: PWM channels, inversion, max output, deadband, controller axes, sim start pose |
| `Drivetrain.java` | The 4 motors, follower setup, `DifferentialDrive`, Field2d, drivetrain physics sim, battery-sag sim |
| `Robot.java` | Only maps the controller to `Drivetrain`: left stick Y → left side, right stick Y → right side |

## Bugs fixed

1. **Follower motors not driven** → `m_leftLeader.addFollower(m_leftFollower)` / `m_rightLeader.addFollower(m_rightFollower)`.
   Inversion is set on the leaders only: `set()` applies the leader's inversion first and then passes the result to its followers (confirmed from the WPILib 2026 bytecode).
2. **Both sides read the same axis** → the left side reads `kLeftYAxis` (1), the right side reads `kRightYAxis` (5).

## Other changes

- `kMaxOutput = 0.5`: **half speed for the first real-robot tests**; raise to 1.0 once directions are verified.
- `kDeadband = 0.05`: the robot does not creep when a worn stick doesn't return exactly to 0.
- `kSquareInputs = true`: finer control at low speed, same top speed.
- Motors are stopped explicitly in `disabledInit()`.
- Dashboard: `Drive/Left Output`, `Drive/Right Output` (read-only), `Field`.
  The `DifferentialDrive` widget is **deliberately not** published with `putData`: it is a writable actuator, so a dashboard can write motor speeds directly and override the `tankDrive` output (also a safety risk on the real robot).
- **Battery sag** in simulation: hard acceleration draws more current and `RobotController.getBatteryVoltage()` drops.
- Everything is logged to a `.wpilog` file; see `TESTING.md`.

## How to run the simulation

In the `pro 2 tank - tank-drive` folder:

```powershell
.\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```

In the Sim GUI:

1. **NetworkTables → SmartDashboard → Field** to open the field window.
2. Set Robot State to **Teleoperated**.
3. Controller (pick one):
   - **Recommended: plug an Xbox controller into the computer.** It appears under System Joysticks; drag it onto Joystick[0]. Left stick = left side, right stick = right side, same as on the real robot.
   - **Keyboard only**: tank drive needs two Y axes (axis 1 and axis 5), but Keyboard 0 only has the first few axes configured by default. Open **DS → Keyboard 0 Settings**, raise the axis count to 6, assign two keys to **Axis 5** (for example I / K), then drag Keyboard 0 onto Joystick[0]. W/S then drives the left side and I/K the right side.

## Verification checklist (go through each row in simulation)

| Action | PWM 3 / 8 (left) | PWM 4 / 5 (right) | On the field |
|---|---|---|---|
| Both sides forward | 3 equals 8, > 0 | 4 equals 5, < 0 (right side inverted) | Drives straight forward |
| Left side only | Non-zero | 0 | Turns right in place (clockwise) |
| Right side only | 0 | Non-zero | Turns left in place (counter-clockwise) |
| Left forward, right backward | > 0 | > 0 | Spins quickly clockwise |
| Switch to Disabled | All 0 | All 0 | Stops |

The maximum value should be **±0.5** (`kMaxOutput`), not ±1.

## Before testing on the real robot (on site)

1. Robot **on blocks**, someone standing by the Space bar.
2. In the DS USB tab, confirm the right stick Y really is **Axis 5**; if not, change `kRightYAxis`.
3. Push the left stick → do both left wheels turn forward? Push the right stick → do both right wheels turn forward?
   - One side reversed → change `kLeftInverted` / `kRightInverted`.
   - The two wheels on the same side turn in opposite directions (fighting each other) → that follower motor is mounted opposite to its leader and needs separate handling. Stop and report it first.
4. Only when everything is correct, put the robot on the ground and drive slowly; then raise `kMaxOutput` to 1.0.

## Known limitations

- The simulated drivetrain uses the **KOP kitbot preset** (2 × CIM per side, 10.71:1, 6 in wheels), not the team's real parameters.
- No encoders or gyro, so no odometry; the field shows the **ground-truth pose** from the physics model.
- No field wall collisions.

## Next steps

- Fill in the real drivetrain parameters (motor type, gear ratio, wheel diameter, mass, track width).
- Add encoders + a gyro → `DifferentialDriveOdometry`, and draw both the "true" and the "estimated" pose on the field.
- Move to Command-based (`Drivetrain` becomes a `SubsystemBase`, controller bindings move to `RobotContainer`).
