/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  // Makes the Compressor
  private final Compressor m_compressor = new Compressor();

  // Makes the Double Solenoid
  private final DoubleSolenoid m_ballStopper = new DoubleSolenoid(PneumaticsConstants.kBallStopperForward, PneumaticsConstants.kBallStopperBackward);

  /**
   * Creates a new Pneumatics.
   */
  public Pneumatics() {
    // Sets up the pneumatics
    m_compressor.setClosedLoopControl(true);
  }

  public DoubleSolenoid getGatePiston(){
    return m_ballStopper;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
