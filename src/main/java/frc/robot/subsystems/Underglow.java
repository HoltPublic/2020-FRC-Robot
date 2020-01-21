/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;;

public class Underglow extends SubsystemBase {
  private final Spark m_blinkin = new Spark(BlinkinConstants.kBlinkinPWM);

  /**
   * Creates a new Underglow.
   */
  public Underglow() {

  }

  public void setColorTeleOP(Alliance color){
    if(color == Alliance.Blue){
      m_blinkin.set(0.87);
    }else if(color == Alliance.Red){
      m_blinkin.set(0.61);
    }else{
      m_blinkin.set(0.93);
    }
  }

  public void setColorAuton(Alliance color){
    if(color == Alliance.Blue){
      m_blinkin.set(-0.09);
    }else if(color == Alliance.Red){
      m_blinkin.set(-0.11);
    }else{
      m_blinkin.set(-0.05);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
