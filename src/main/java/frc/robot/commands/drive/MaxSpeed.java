/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MaxSpeed extends CommandBase {
  private final DriveSubsystem m_drive;
  private final ShuffleboardTab m_tab;
  private double testingSpeed;
  private double maxSpeed = 0;

  /**
   * Creates a new MaxSpeed.
   */
  public MaxSpeed(ShuffleboardTab tab, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tab = tab;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testingSpeed = m_drive.getNavX().getVelocityX();
    if(testingSpeed > maxSpeed){
      maxSpeed = testingSpeed;
      m_tab.addNumber("Max Speed", () -> maxSpeed);
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
