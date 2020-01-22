/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Underglow;

public class AutonLights extends CommandBase {
  private final Underglow m_glow;

  /**
   * Creates a new AutonLights.
   */
  public AutonLights(Underglow glow) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_glow = glow;
    addRequirements(m_glow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_glow.setColorAuton(DriverStation.getInstance().getAlliance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !DriverStation.getInstance().isAutonomous();
  }
}
