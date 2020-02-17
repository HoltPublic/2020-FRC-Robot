/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorsensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;

public class RotationControl extends CommandBase {
  private final ColorSensor m_color;

  /**
   * Creates a new RotationControl.
   */
  public RotationControl(ColorSensor color) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_color = color;
    addRequirements(m_color);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_color.getMotor().set(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_color.getMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_color.getTotalColorWheelSpins()) >= 4;
  }
}
