/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Motors on the Right Side
  private final WPI_VictorSPX m_rightMasterMotor = new WPI_VictorSPX(DriveConstants.kRightMotor1Port);
  private final WPI_VictorSPX m_rightSlaveMotor = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);

  // Motors on the Left Side
  private final WPI_VictorSPX m_leftMasterMotor = new WPI_VictorSPX(DriveConstants.kLeftMotor1Port);
  private final WPI_VictorSPX m_leftSlaveMotor = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);

  // The Robot's Drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMasterMotor, m_rightMasterMotor);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets up the Talons
    m_rightMasterMotor.configFactoryDefault();
    m_rightSlaveMotor.configFactoryDefault();

    m_leftMasterMotor.configFactoryDefault();
    m_leftSlaveMotor.configFactoryDefault();

    m_leftSlaveMotor.follow(m_leftMasterMotor);
    m_rightSlaveMotor.follow(m_rightMasterMotor);

    m_rightMasterMotor.setInverted(false);
    m_rightSlaveMotor.setInverted(InvertType.FollowMaster);

    m_leftMasterMotor.setInverted(false);
    m_leftSlaveMotor.setInverted(InvertType.FollowMaster);

    m_drive.setRightSideInverted(true);
  }

  // Drives the robot
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  // Sets the Max seed of the robot
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  // Stops the robot
  public void stopDrive(){
    m_drive.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
