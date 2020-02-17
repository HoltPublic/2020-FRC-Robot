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
  private final WPI_VictorSPX m_beltIntake = new WPI_VictorSPX(IntakeConstants.kBeltMotor);

  /**
   * Creates a new Intake.
   */
  public Intake() {
    // Sets up the motor
    m_beltIntake.configFactoryDefault();

    m_beltIntake.setInverted(false);
  }

  public WPI_VictorSPX getIntakeMotor(){
    return m_beltIntake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
