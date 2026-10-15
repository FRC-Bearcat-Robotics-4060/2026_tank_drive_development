# 2026 Tank Drive Development — FRC Team 4060 (Bearcat Robotics)

Java robot code for Team 4060's tank-drive (differential drive) robot, built with **WPILib 2026**.

This branch (`main`) is the **original baseline** exactly as it came from the team computer. Development happens on separate branches (see [Branches](#branches)).

## Hardware

| Item | Value |
|---|---|
| Controller | roboRIO (image `FRC_roboRIO_2026_v1.2`) |
| Drive motors | 4 × motor controllers driven by `PWMSparkMax`, 2 per side |
| Left side | PWM **3** (leader), PWM **8** (follower) |
| Right side | PWM **4** (leader, inverted), PWM **5** (follower) |
| Driver input | Joystick on Driver Station port **0** |
| Team number | 4060 (roboRIO at `10.40.60.2`, USB `172.22.11.2`) |

## Software

| Item | Version |
|---|---|
| WPILib / GradleRIO | 2026.2.1 |
| Java | 17 (WPILib JDK at `C:\Users\Public\wpilib\2026\jdk`) |
| Vendor libraries | REVLib 2026.0.5, CTRE Phoenix 6 26.3.0, WPILib New Commands (installed, not used yet) |

## Getting started

1. Install **WPILib 2026** (includes VS Code, JDK 17 and the simulator):
   https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
2. Clone this repo and open the folder that contains `build.gradle` in **2026 WPILib VS Code**.
3. Use the command palette (`Ctrl+Shift+P`):

| Task | Command palette | Terminal equivalent |
|---|---|---|
| Build | `WPILib: Build Robot Code` | `./gradlew build` |
| Simulate | `WPILib: Simulate Robot Code` → check **Sim GUI** only | `./gradlew simulateJava` |
| Deploy to robot | `WPILib: Deploy Robot Code` (`Shift+F5`) | `./gradlew deploy` |

> **Do not** use the *Run / Debug* buttons above `main()`. They start the program with the plain Java launcher, which does not load WPILib's native libraries and fails with `wpiutiljni could not be loaded from path`.

If you are not using the WPILib VS Code, point Gradle at the WPILib JDK:

```powershell
.\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk"
```

## Known issues in this baseline

These are intentionally left unfixed on `main` so the original behavior is preserved:

1. **Follower motors are never driven.** `SendableRegistry.addChild()` only registers the followers for the dashboard; PWM 8 and PWM 5 always output 0, so each side runs on one motor. Fix: `leader.addFollower(follower)`.
2. **The robot cannot turn.** `tankDrive(-m_stick.getY(), -m_stick.getY())` sends the same stick axis to both sides. Fix: read a separate axis for the right side (e.g. axis 5, the right stick Y on an Xbox-style controller).

Both are visible in simulation without a robot: in the Sim GUI, hold `W` on Keyboard 0 and watch the **PWM Outputs** panel (PWM 3 and 4 move, PWM 5 and 8 stay at 0).

## Branches

Development branches (pushed separately once reviewed):

| Branch | Purpose |
|---|---|
| `main` | Original baseline from the team computer |
| `sim-field` | Adds drivetrain physics simulation and a Field2d view; driving code unchanged (bugs kept for demonstration) |
| `tank-drive` | Fixes both bugs, splits code into `Constants` / `Drivetrain` / `Robot`, half-speed limit for first tests, `.wpilog` data logging and an offline log analyzer (`tools/analyze_drive_log.py`) |

## Safety

- Successful deploy is **not** permission to enable. Put the robot **on blocks** (wheels off the ground) for the first test of any new code.
- Someone must stand at the Driver Station ready to press **Space** (E-stop) / **Enter** (disable).
- Never enable the robot over remote desktop; the person at the robot enables it.

## Useful references

- WPILib docs: https://docs.wpilib.org
- Robot simulation: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html
- Driver Station errors and warnings: https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station-errors-warnings.html

## License

Code in this repository is released under the [MIT License](LICENSE). WPILib template files are covered by the [WPILib BSD license](WPILib-License.md).
