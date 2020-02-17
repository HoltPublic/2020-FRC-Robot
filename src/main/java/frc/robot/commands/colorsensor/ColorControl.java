/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorsensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;

public class ColorControl extends CommandBase {
  private final ColorSensor m_color;
  private String m_gameData;

  /**
   * Creates a new ColorControl.
   */
  public ColorControl(ColorSensor color) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_color = color;
    addRequirements(m_color);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gameData = m_color.getGameData();
    m_color.getMotor().set(.5);
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
    if(m_color.getColor().charAt(0) == m_gameData.charAt(0)){
      return true;
    } else{
      return false;
    }
  }
}
