/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;;

public class Underglow extends SubsystemBase {
  // Makes the Blinkin
  private final Spark m_blinkin = new Spark(BlinkinConstants.kBlinkinPWM);

  /**
   * Creates a new Underglow.
   */
  public Underglow() {

  }

  // Sets the robot to the alliance color
  public void setColorTeleOP(Alliance color){
    if(color == Alliance.Blue){
      m_blinkin.set(0.87);
    }else if(color == Alliance.Red){
      m_blinkin.set(0.61);
    }else{
      m_blinkin.set(0.93);
    }
  }

  // Sets the robot to the alliance color but blinking
  public void setColorAuton(Alliance color){
    if(color == Alliance.Blue){
      m_blinkin.set(-0.09);
    }else if(color == Alliance.Red){
      m_blinkin.set(-0.11);
    }else{
      m_blinkin.set(-0.05);
    }
  }

  // Sets the robot to the Colorwheel color
  public void setColorWheelColor(Color color){
    if(color == Color.kBlue){
      m_blinkin.set(-0.09);
    } else if(color == Color.kGreen){
      m_blinkin.set(0.77);
    } else if(color == Color.kRed){
      m_blinkin.set(0.61);
    } else if(color == Color.kYellow){
      m_blinkin.set(0.69);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
