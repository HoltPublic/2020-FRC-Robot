/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Lift;

public class LightSaberUp extends CommandBase {
  private final Lift m_lift;

  /**
   * Creates a new LightSaberUp.
   */
  public LightSaberUp(Lift lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Resets the encoder
    m_lift.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //lifts the telecope
    m_lift.getTelescopeMotor().set(LiftConstants.kTelescopeSpeedUp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the motor
    m_lift.getTelescopeMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Checks to see if the telescope is at the right height
    return Math.abs(m_lift.getRotation()) >= LiftConstants.kRotationsToGoUp;
  }
}
