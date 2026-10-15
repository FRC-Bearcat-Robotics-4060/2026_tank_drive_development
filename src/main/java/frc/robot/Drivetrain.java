package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;

/**
 * Tank drivetrain: 2 PWM SPARK MAX per side (one leader, one follower).
 *
 * <p>Owns the motors, the DifferentialDrive helper, the dashboard field view and the drivetrain
 * physics simulation. The rest of the robot only calls {@link #tankDrive}, {@link #stop},
 * {@link #periodic} and {@link #simulationPeriodic}.
 */
public class Drivetrain {
  private final PWMSparkMax m_leftLeader = new PWMSparkMax(DriveConstants.kLeftLeaderPwm);
  private final PWMSparkMax m_leftFollower = new PWMSparkMax(DriveConstants.kLeftFollowerPwm);
  private final PWMSparkMax m_rightLeader = new PWMSparkMax(DriveConstants.kRightLeaderPwm);
  private final PWMSparkMax m_rightFollower = new PWMSparkMax(DriveConstants.kRightFollowerPwm);

  private final DifferentialDrive m_drive;

  /** Field view, published as "SmartDashboard/Field". */
  private final Field2d m_field = new Field2d();

  /**
   * Drivetrain physics model, only created in simulation (null on the real robot).
   *
   * <p>ASSUMPTION: KOP kitbot preset (2 CIM per side, 10.71:1, 6 inch wheels). Replace with the
   * real drivetrain parameters, otherwise simulated speed/acceleration will not match the robot.
   */
  private final DifferentialDrivetrainSim m_driveSim;

  public Drivetrain() {
    // Inversion goes on the leaders only (see DriveConstants.kRightInverted for why).
    m_leftLeader.setInverted(DriveConstants.kLeftInverted);
    m_rightLeader.setInverted(DriveConstants.kRightInverted);

    // Followers copy whatever their leader outputs. This was the missing piece in the original
    // code: SendableRegistry.addChild() only affects the dashboard, it never drove PWM 5 and 8.
    m_leftLeader.addFollower(m_leftFollower);
    m_rightLeader.addFollower(m_rightFollower);

    m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);
    m_drive.setMaxOutput(DriveConstants.kMaxOutput);
    m_drive.setDeadband(DriveConstants.kDeadband);

    // Deliberately NOT publishing m_drive with SmartDashboard.putData(): DifferentialDrive is an
    // actuator Sendable whose "Left/Right Motor Speed" entries are writable and call the motors'
    // set() directly. A dashboard widget (e.g. the Sim GUI "Drive" window) can then overwrite what
    // tankDrive() commanded, and on the real robot anything on the network could drive the motors.
    // Read-only numbers are published in periodic() instead.
    SmartDashboard.putData("Field", m_field);

    if (RobotBase.isSimulation()) {
      m_driveSim =
          DifferentialDrivetrainSim.createKitbotSim(
              KitbotMotor.kDualCIMPerSide,
              KitbotGearing.k10p71,
              KitbotWheelSize.kSixInch,
              null); // null = no measurement noise
      m_driveSim.setPose(SimConstants.kStartPose);
      m_field.setRobotPose(SimConstants.kStartPose);
    } else {
      m_driveSim = null;
    }
  }

  /**
   * Tank drive: each value drives one side directly.
   *
   * @param leftSpeed left side, -1..1, positive = forward
   * @param rightSpeed right side, -1..1, positive = forward
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed, DriveConstants.kSquareInputs);
  }

  /** Stop all drive motors immediately. */
  public void stop() {
    m_drive.stopMotor();
  }

  /** Call once per loop from Robot.robotPeriodic(), in every mode. */
  public void periodic() {
    // get() is the value before inversion: +1 = that side forward, on both sides.
    SmartDashboard.putNumber("Drive/Left Output", m_leftLeader.get());
    SmartDashboard.putNumber("Drive/Right Output", m_rightLeader.get());
  }

  /** Call once per loop from Robot.simulationPeriodic(). Does nothing on the real robot. */
  public void simulationPeriodic() {
    if (m_driveSim == null) {
      return;
    }

    // 1. What the code commands each side to do, as a voltage. Disabled = outputs off = 0 V.
    double batteryVolts = RobotController.getBatteryVoltage();
    double leftVolts = 0.0;
    double rightVolts = 0.0;
    if (DriverStation.isEnabled()) {
      leftVolts = m_leftLeader.get() * batteryVolts;
      rightVolts = m_rightLeader.get() * batteryVolts;
    }

    // 2. Advance the physics by one robot loop.
    m_driveSim.setInputs(leftVolts, rightVolts);
    m_driveSim.update(TimedRobot.kDefaultPeriod);

    // 3. Battery sag: hard acceleration draws current and pulls the battery voltage down,
    //    just like on the real robot. Shows up as a lower RobotController.getBatteryVoltage().
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveSim.getCurrentDrawAmps()));

    // 4. Draw the simulated (ground-truth) pose.
    m_field.setRobotPose(m_driveSim.getPose());
  }

  /** Simulated ground-truth pose (simulation only; returns null on the real robot). */
  public Pose2d getSimPose() {
    return m_driveSim == null ? null : m_driveSim.getPose();
  }
}
