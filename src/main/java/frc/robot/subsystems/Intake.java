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
  // Makes the intake motor
  private final WPI_VictorSPX m_BeltIntake = new WPI_VictorSPX(IntakeConstants.kBeltMotor);

  // the max speed
  private final double maxSpeed = 0.6;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    // Sets up the motor
    m_BeltIntake.configFactoryDefault();

    m_BeltIntake.setInverted(false);
  }

  // Starts the intake forward
  public void intakeTheBalls(){
    m_BeltIntake.set(maxSpeed);
  }

  // Starts the intake backwards
  public void spitTheBalls(){
    m_BeltIntake.set(-maxSpeed);
  }

  // Stops the intake
  public void stopMotor(){
    m_BeltIntake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
