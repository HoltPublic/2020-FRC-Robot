/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTime extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_time;
  private final double m_speed;
  private final Timer m_timer = new Timer();
  /**
   * Creates a new DriveTime.
   */
  public DriveTime(double time, double speed, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_time = time;
    m_speed = speed;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stops the robot incase it was driving
    m_drive.stopDrive();
    // Sets the timer to 0
    m_timer.reset();
    // Starts the timer
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drives the robot at the set speed
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // After the timer is done stop the robot
    m_drive.stopDrive();
    // Stops the timer the save CPU Power
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Keeps checking if the set time has passed
    return m_timer.hasPeriodPassed(m_time);
  }
}
