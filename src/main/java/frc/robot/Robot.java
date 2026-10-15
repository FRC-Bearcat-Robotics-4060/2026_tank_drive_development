// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 *
 * <p>Branch sim-field: adds a physics simulation of the drivetrain and draws the robot on a field
 * (Field2d). The driving code itself is unchanged, including its two known bugs (followers not
 * driven, both sides read the same axis), so the simulation shows exactly what the real robot
 * would do with this code.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private Joystick m_stick;

  private final PWMSparkMax m_leftleaderMotor = new PWMSparkMax(3);
  private final PWMSparkMax m_rightleaderMotor = new PWMSparkMax(4);
  private final PWMSparkMax m_leftfollowerMotor = new PWMSparkMax(8);
  private final PWMSparkMax m_rightfollowerMotor = new PWMSparkMax(5);

  // ---------------------------------------------------------------------------------------------
  // Field simulation
  // ---------------------------------------------------------------------------------------------

  /**
   * Where the simulated robot starts on the field, in meters, measured from the blue-alliance
   * origin (bottom-left corner of the field image). Rotation 0 = facing the red alliance wall.
   */
  private static final Pose2d kSimStartPose = new Pose2d(2.0, 4.0, Rotation2d.kZero);

  /**
   * Field widget. Published to NetworkTables as "SmartDashboard/Field"; the Sim GUI, Glass,
   * Elastic and AdvantageScope can all draw it. Created on the real robot too (harmless), so that
   * odometry can be drawn on the same widget later.
   */
  private final Field2d m_field = new Field2d();

  /**
   * Physics model of the drivetrain. Only created in simulation (see simulationInit()).
   *
   * <p>ASSUMPTION: the real drivetrain parameters are unknown, so this uses the KOP "kitbot"
   * preset: 2 CIM motors per side, 10.71:1 gearbox, 6 inch wheels. Replace with the team's real
   * motor type, gear ratio, wheel size, mass and track width once known, otherwise speeds and
   * accelerations in simulation will not match the real robot.
   */
  private DifferentialDrivetrainSim m_driveSim;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightleaderMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftleaderMotor::set, m_rightleaderMotor::set);
    m_stick = new Joystick(0);

    SendableRegistry.addChild(m_robotDrive, m_leftfollowerMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightfollowerMotor);

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_stick.getY(), -m_stick.getY());
  }

  /** Runs once when the program starts, only in simulation. */
  @Override
  public void simulationInit() {
    m_driveSim =
        DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDualCIMPerSide,
            KitbotGearing.k10p71,
            KitbotWheelSize.kSixInch,
            null); // null = no measurement noise
    m_driveSim.setPose(kSimStartPose);
    m_field.setRobotPose(kSimStartPose);
  }

  /** Runs every loop (after the mode-specific periodic method), only in simulation. */
  @Override
  public void simulationPeriodic() {
    // 1. Read what the code is commanding each side to do.
    //    get() returns the value BEFORE the motor's inversion is applied, i.e. +1 means "this side
    //    forward" for both sides. That is the convention DifferentialDrivetrainSim expects.
    //    Only the leaders are read: the followers are not driven by this code (known bug), so the
    //    model is fed exactly what the leader PWM channels output.
    // 2. Convert duty cycle (-1..1) to volts using the (simulated) battery voltage.
    // 3. When disabled the real robot's PWM outputs are off, so feed 0 V.
    double batteryVolts = RobotController.getBatteryVoltage();
    double leftVolts = 0.0;
    double rightVolts = 0.0;
    if (DriverStation.isEnabled()) {
      leftVolts = m_leftleaderMotor.get() * batteryVolts;
      rightVolts = m_rightleaderMotor.get() * batteryVolts;
    }

    // 4. Advance the physics by one robot loop (20 ms by default).
    m_driveSim.setInputs(leftVolts, rightVolts);
    m_driveSim.update(getPeriod());

    // 5. Draw the simulated (ground-truth) pose on the field.
    m_field.setRobotPose(m_driveSim.getPose());
  }
}
