/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
  // Makes the climb motor
  private final WPI_VictorSPX m_winchMotor = new WPI_VictorSPX(LiftConstants.kClimbMotor);

  // Makes the telescope motor
  public final WPI_TalonSRX m_telescopeMotor = new WPI_TalonSRX(LiftConstants.kTelescopeMotor);

  /**
   * Creates a new Lift.
   */
  public Lift() {
    // Sets up the motors
    m_winchMotor.configFactoryDefault();
    m_telescopeMotor.configFactoryDefault();

    m_winchMotor.setInverted(true);
    m_telescopeMotor.setInverted(false);

    m_winchMotor.setNeutralMode(NeutralMode.Brake);

    // Sets up the encoder
    m_telescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public WPI_VictorSPX getWinchMotor(){
    return m_winchMotor;
  }

  public WPI_TalonSRX getTelescopeMotor(){
    return m_telescopeMotor;
  }

  // Resets the encoder
  public void resetEncoder(){
    m_telescopeMotor.setSelectedSensorPosition(0);
  }

  // Gets the rotation from the encoder
  public double getRotation(){
    return m_telescopeMotor.getSelectedSensorPosition() / LiftConstants.kEncoderCPR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
