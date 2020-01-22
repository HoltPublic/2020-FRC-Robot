/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Underglow;

public class TeleOPLights extends CommandBase {
  private final Underglow m_glow;
  private String gameData;
  /**
   * Creates a new TeleOPLights.
   */
  public TeleOPLights(Underglow glow) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_glow = glow;
    addRequirements(m_glow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0){
      switch(gameData.charAt(0)){
        case 'B' : m_glow.setColorWheelColor(Color.kBlue); break;
        case 'G' : m_glow.setColorWheelColor(Color.kGreen); break;
        case 'R' : m_glow.setColorWheelColor(Color.kRed); break;
        case 'Y' : m_glow.setColorWheelColor(Color.kYellow); break;
      }
    } else{
      m_glow.setColorTeleOP(DriverStation.getInstance().getAlliance());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
