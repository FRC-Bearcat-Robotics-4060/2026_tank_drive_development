// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private Joystick m_stick;

  private final PWMSparkMax m_leftleaderMotor = new PWMSparkMax(3);
  private final PWMSparkMax m_rightleaderMotor = new PWMSparkMax(4);
  private final PWMSparkMax m_leftfollowerMotor = new PWMSparkMax(8);
  private final PWMSparkMax m_rightfollowerMotor = new PWMSparkMax(5);

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
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_stick.getY(), -m_stick.getY());
  }

}
