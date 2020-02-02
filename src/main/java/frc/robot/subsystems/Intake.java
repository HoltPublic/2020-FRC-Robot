/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX m_BeltIntake = new WPI_VictorSPX(IntakeConstants.kBeltMotor);
  double maxSpeed = 0.6;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    m_BeltIntake.configFactoryDefault();

    m_BeltIntake.setInverted(false);
  }

  public void intakeTheBalls(){
    m_BeltIntake.set(maxSpeed);
  }

  public void spitTheBalls(){
    m_BeltIntake.set(-maxSpeed);
  }

  public void stopMotor(){
    m_BeltIntake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
