// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.OIConstants;

/**
 * Team 4060 tank-drive robot.
 *
 * <p>Robot only wires the driver controller to the {@link Drivetrain}; everything about motors,
 * inversion, followers and simulation lives in Drivetrain, and every number lives in
 * {@link Constants}.
 */
public class Robot extends TimedRobot {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);

  /** Runs every loop in every mode (disabled, auto, teleop, test). */
  @Override
  public void robotPeriodic() {
    m_drivetrain.periodic();
  }

  /** Make sure nothing keeps moving when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_drivetrain.stop();
  }

  @Override
  public void teleopPeriodic() {
    // Joystick Y axes are -1 when pushed forward, so negate: forward on the stick = +1 = forward.
    double left = -m_driver.getRawAxis(OIConstants.kLeftYAxis);
    double right = -m_driver.getRawAxis(OIConstants.kRightYAxis);
    m_drivetrain.tankDrive(left, right);
  }

  /** Runs every loop, only in simulation. */
  @Override
  public void simulationPeriodic() {
    m_drivetrain.simulationPeriodic();
  }
}
