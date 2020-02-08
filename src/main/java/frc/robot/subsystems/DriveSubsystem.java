/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Motors on the Right Side
  private final WPI_TalonFX m_rightMasterMotor = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_rightSlaveMotor = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  // Motors on the Left Side
  private final WPI_TalonFX m_leftMasterMotor = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_leftSlaveMotor = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);

  // The Robot's Drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMasterMotor, m_rightMasterMotor);

  // The navX
  private final AHRS m_navX = new AHRS(SPI.Port.kMXP);

  // So the Falcons can sing
  private final Orchestra m_orchestra = new Orchestra();
  

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

    m_rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    m_rightSlaveMotor.setNeutralMode(NeutralMode.Brake);
    
    m_leftMasterMotor.setNeutralMode(NeutralMode.Brake);
    m_leftSlaveMotor.setNeutralMode(NeutralMode.Brake);

    m_rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_drive.setRightSideInverted(true);

    m_orchestra.addInstrument(m_rightMasterMotor);
    m_orchestra.addInstrument(m_rightSlaveMotor);

    m_orchestra.addInstrument(m_leftMasterMotor);
    m_orchestra.addInstrument(m_leftSlaveMotor);

    m_orchestra.loadMusic("song.chrp");
  }

  // Drives the robot
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  // Sets the Max seed of the robot
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  // Flips the front of the robot
  public void flipDrive(double fwd, double rot){
    m_drive.arcadeDrive(-fwd, rot);
  }

  // Stops the robot
  public void stopDrive(){
    m_drive.stopMotor();
  }

  // Gets the average distance from both sides
  public double getDistance(){
    return (getRightDistance() + getLeftDistance()) / 2;
  }

  // Gets how far the right motors have gone
  public double getRightDistance(){
    return -((m_rightMasterMotor.getSelectedSensorPosition() / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumference;
  }

  // Gets how far the left motors have gone
  public double getLeftDistance(){
    return ((m_leftMasterMotor.getSelectedSensorPosition() / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumference;
  }

  // Sets the encoders to 0
  public void resetEncoders(){
    m_rightMasterMotor.setSelectedSensorPosition(0);
    m_leftMasterMotor.setSelectedSensorPosition(0);
  }

  // Returns the angle of the gyro
  public double getGyro(){
    return m_navX.getAngle();
  }

  // Resets the gyro to 0
  public void resetGyro(){
    m_navX.reset();
  }

  // Starts the music
  public void playMusic(){
    m_orchestra.play();
  }

  // Stops the music
  public void stopMusic(){
    m_orchestra.stop();
  }

  // Checks to see if the music is playing
  public boolean isMusicPlaying(){
    return m_orchestra.isPlaying();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Adds the drive to the computer
  }
}
