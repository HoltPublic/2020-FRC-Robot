/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnAngleLeft extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_speed;
  private final double m_angle;

  /**
   * Creates a new TurnAngle.
   */
  public TurnAngleLeft(double angle, double speed, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = speed;
    m_angle = angle;
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stops the robot
    m_drive.stopDrive();
    // Resets the gyro
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Spins the robot
    m_drive.arcadeDrive(0, -m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the robot
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(-m_drive.getGyro()) >= m_angle;
  }
}
