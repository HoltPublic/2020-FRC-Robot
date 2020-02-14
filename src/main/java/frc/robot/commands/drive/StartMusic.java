/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class StartMusic extends CommandBase {
  private final DriveSubsystem m_drive;
  private final String m_song;

  /**
   * Creates a new StartMusic.
   */
  public StartMusic(String song, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_song = song;
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.getOrchestra().loadMusic(m_song);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.getOrchestra().play();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.getOrchestra().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_drive.getOrchestra().isPlaying();
  }
}
