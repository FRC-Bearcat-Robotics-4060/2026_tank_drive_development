// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private Joystick m_stick;

  private final SparkMax m_leftleaderMotor = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_rightleaderMotor = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax m_leftfollowerMotor = new SparkMax(8, MotorType.kBrushed);
  private final SparkMax m_rightfollowerMotor = new SparkMax(5, MotorType.kBrushed);

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
    // m_leftleaderMotor.addFolloPwer(m_leftfollowerMotor);
    // m_rightleaderMotor.addFollower(m_rightfollowerMotor);
    m_leftfollowerMotor.configure(new SparkMaxConfig().follow(m_leftleaderMotor),
    ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightfollowerMotor.configure(new SparkMaxConfig().follow(m_rightleaderMotor),
    ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  
  
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_stick.getY(), -m_stick.getY());
    //m_robotDrive.tankDrive(-m_stick.getY(), -m_stick.getRawAxis(5));
  }

}
