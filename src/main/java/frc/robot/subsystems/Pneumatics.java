/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  private final Compressor m_compressor = new Compressor();
  private final DoubleSolenoid m_ballStopper = new DoubleSolenoid(PneumaticsConstants.kBallStopperForward, PneumaticsConstants.kBallStopperBackward);

  /**
   * Creates a new Pneumatics.
   */
  public Pneumatics() {
    m_compressor.setClosedLoopControl(true);
  }

  public void stopBalls(){
    m_ballStopper.set(Value.kForward);
  }

  public void goBalls(){
    m_ballStopper.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}