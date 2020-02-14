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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Motors on the Right Side
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_rightSlave = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  // Motors on the Left Side
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_leftSlave = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);

  // The Robot's Drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  // The navX
  private final AHRS m_navX = new AHRS(SPI.Port.kMXP);

  // So the Falcons can sing
  public final Orchestra m_orchestra = new Orchestra();

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.kTrackWidthInches));
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());

  private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kP, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kP, 0, 0);
  
  private Pose2d m_pose;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets up the Talons
    m_rightMaster.configFactoryDefault();
    m_rightSlave.configFactoryDefault();

    m_leftMaster.configFactoryDefault();
    m_leftSlave.configFactoryDefault();

    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    m_rightMaster.setInverted(false);
    m_rightSlave.setInverted(InvertType.FollowMaster);

    m_leftMaster.setInverted(false);
    m_leftSlave.setInverted(InvertType.FollowMaster);

    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightSlave.setNeutralMode(NeutralMode.Brake);
    
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftSlave.setNeutralMode(NeutralMode.Brake);

    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_drive.setRightSideInverted(true);

    m_orchestra.addInstrument(m_rightMaster);
    m_orchestra.addInstrument(m_rightSlave);

    m_orchestra.addInstrument(m_leftMaster);
    m_orchestra.addInstrument(m_leftSlave);
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

  public DifferentialDrive getDrive(){
    return m_drive;
  }

  // Gets the average distance from both sides
  public double getDistanceInches(){
    return (getRightDistanceInches() + getLeftDistanceInches()) / 2;
  }

  // Gets how far the right motors have gone
  public double getRightDistanceInches(){
    return -((m_rightMaster.getSelectedSensorPosition() / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceInches;
  }

  // Gets how far the left motors have gone
  public double getLeftDistanceInches(){
    return ((m_leftMaster.getSelectedSensorPosition() / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceInches;
  }

  // Sets the encoders to 0
  public void resetEncoders(){
    m_rightMaster.setSelectedSensorPosition(0);
    m_leftMaster.setSelectedSensorPosition(0);
  }

  // Returns the angle of the gyro
  public double getGyro(){
    return m_navX.getAngle();
  }

  // Resets the gyro to 0
  public void resetGyro(){
    m_navX.reset();
  }

  public Orchestra getOrchestra(){
    return m_orchestra;
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-m_navX.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public double getLeftVelocity(){
    return m_leftMaster.getSelectedSensorVelocity() * DriveConstants.kMetersPerTick;
  }

  public double getRightVelocity(){
    return -m_rightMaster.getSelectedSensorVelocity() * DriveConstants.kMetersPerTick;
  }

  public double getLeftDistanceMeter(){
    return m_leftMaster.getSelectedSensorPosition() * DriveConstants.kMetersPerTick;
  }

  public double getRightDistanceMeter(){
    return -m_rightMaster.getSelectedSensorPosition() * DriveConstants.kMetersPerTick;
  }

  public SimpleMotorFeedforward getFeedforward(){
    return m_feedForward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public PIDController getLeftPIDController(){
    return m_leftPIDController;
  }

  public PIDController getRightPIDController(){
    return m_rightPIDController;
  }

  public Pose2d getPose(){
    return m_pose;
  }

  public void setOutput(double leftVolts, double rightVolts){
    m_leftMaster.set(leftVolts / 12);
    m_rightMaster.set(rightVolts / 12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pose = m_odometry.update(getHeading(), getLeftDistanceMeter(), getRightDistanceMeter());
  }
}
