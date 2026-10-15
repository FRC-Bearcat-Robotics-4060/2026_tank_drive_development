package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * All robot-wide constants in one place. Wiring, inversions and tuning live here so that a wiring
 * mismatch found on the real robot is a one-line change instead of a hunt through the code.
 */
public final class Constants {
  private Constants() {}

  /** Drivetrain wiring and behavior. */
  public static final class DriveConstants {
    private DriveConstants() {}

    // ---- roboRIO PWM channels (must match the real wiring) ----
    public static final int kLeftLeaderPwm = 3;
    public static final int kLeftFollowerPwm = 8;
    public static final int kRightLeaderPwm = 4;
    public static final int kRightFollowerPwm = 5;

    // ---- Inversion ----
    // One side is mounted mirrored, so it must be inverted for "+1 = forward" on both sides.
    // Only the LEADERS are inverted: PWMMotorController.set() applies the leader's inversion and
    // then passes the already-inverted value to its followers. Inverting a follower as well would
    // make it fight its leader (unless that follower is physically mounted the other way).
    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;

    // ---- Driver feel / safety ----
    /**
     * Output scale applied to everything DifferentialDrive sends to the motors (0..1). Kept at 0.5
     * for the first real-robot tests; raise to 1.0 once direction and wiring are verified.
     */
    public static final double kMaxOutput = 0.5;

    /** Stick values smaller than this are treated as 0 (worn sticks rarely rest exactly at 0). */
    public static final double kDeadband = 0.05;

    /** Square the stick inputs: finer control at low speed, same top speed. */
    public static final boolean kSquareInputs = true;
  }

  /** Driver controller mapping. */
  public static final class OIConstants {
    private OIConstants() {}

    public static final int kDriverControllerPort = 0;

    // Axis numbers for an Xbox-style controller (Xbox, Logitech F310 in X mode, most gamepads).
    // Check them in the Driver Station USB tab: push each stick and watch which axis moves.
    public static final int kLeftYAxis = 1;
    public static final int kRightYAxis = 5;
  }

  /** Values used only by the drivetrain simulation. */
  public static final class SimConstants {
    private SimConstants() {}

    /**
     * Where the simulated robot starts, in meters from the blue-alliance origin (bottom-left of
     * the field image). Rotation 0 = facing the red alliance wall.
     */
    public static final Pose2d kStartPose = new Pose2d(2.0, 4.0, Rotation2d.kZero);

    // The physics model currently uses WPILib's KOP "kitbot" preset (see Drivetrain). These are
    // placeholders until the team's real motor type, gear ratio, wheel size, mass and track width
    // are known.
  }
}
