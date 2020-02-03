/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
  private final WPI_VictorSPX m_climbMotor = new WPI_VictorSPX(LiftConstants.kClimbMotor);
  private final WPI_TalonSRX m_telescope = new WPI_TalonSRX(LiftConstants.kTelescopeMotor);
  /**
   * Creates a new Lift.
   */
  public Lift() {
    m_climbMotor.configFactoryDefault();
    m_telescope.configFactoryDefault();

    m_climbMotor.setInverted(false);
    m_telescope.setInverted(false);

    m_telescope.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public void raiseTelescope(){
    m_telescope.set(1);
  }

  public void lowerelescope(){
    m_telescope.set(-1);
  }

  public void stopTelescope(){
    m_telescope.set(0);
  }

  public void resetEncoder(){
    m_telescope.setSelectedSensorPosition(0);
  }

  public double getRotation(){
    return m_telescope.getSelectedSensorPosition() / LiftConstants.kEncoderCPR;
  }

  public void startWinch(){
    m_climbMotor.set(1);
  }

  public void stopWinch(){
    m_climbMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}