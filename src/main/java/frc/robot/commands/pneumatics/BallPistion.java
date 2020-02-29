/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class BallPistion extends CommandBase {
  private final Pneumatics m_blow;
  /**
   * Creates a new BallPistion.
   */
  public BallPistion(Pneumatics blow) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_blow = blow;
    addRequirements(m_blow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Blocks the balls
    m_blow.getGatePiston().set(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Let the balls go through
    m_blow.getGatePiston().set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
