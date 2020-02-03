/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  /**
   * Creates a new DriveDistance.
   */
  public DriveDistance(double distanceInches, double speed, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_distance = distanceInches;
    m_speed = speed;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_drive.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getDistance()) >= m_distance;
  }
}
