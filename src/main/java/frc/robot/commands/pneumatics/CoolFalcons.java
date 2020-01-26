/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pneumatics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.subsystems.Pneumatics;

public class CoolFalcons extends CommandBase {
  private final Pneumatics m_blow;
  private final Timer m_timerForLooping = new Timer();
  private final Timer m_timerForCooling = new Timer();
  /**
   * Creates a new CoolFalcons.
   */
  public CoolFalcons(Pneumatics blow) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_blow = blow;
    addRequirements(m_blow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timerForLooping.reset();
    m_timerForLooping.start();
    m_timerForCooling.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timerForLooping.hasPeriodPassed(PneumaticsConstants.kPneumaticsCoolWaitSeconds)){
      m_blow.startCooling();
      m_timerForCooling.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_blow.stopCooling();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timerForCooling.hasPeriodPassed(PneumaticsConstants.kPneumaticsCoolTimeOutSeconds);
  }
}
