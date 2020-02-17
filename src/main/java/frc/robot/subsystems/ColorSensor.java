/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorConstants;

public class ColorSensor extends SubsystemBase {
  // The Motor
  private final WPI_TalonSRX m_colorMotor = new WPI_TalonSRX(ColorConstants.kColorMotor);

  // Makes the I2C port
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  // Makes the ColorSensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // All the Colormatch Stuff
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  /**
   * Creates a new ColorSensor.
   */
  public ColorSensor() {
    // Adds the colors to the colormatcher
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    // Sets the motor up
    m_colorMotor.configFactoryDefault();
    m_colorMotor.setInverted(false);
    m_colorMotor.setNeutralMode(NeutralMode.Brake);
    m_colorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public String getColor(){
    // gets the current color
    ColorMatchResult m_result = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());

    // Checks the color and returns it
    if (m_result.color == kBlueTarget) {
      return "Blue";
    } else if (m_result.color == kRedTarget) {
      return "Red";
    } else if (m_result.color == kGreenTarget) {
      return "Green";
    } else if (m_result.color == kYellowTarget) {
      return "Yellow";
    } else {
      return "Unknown";
    }
  }

  public double getTotalColorWheelSpins(){
    return m_colorMotor.getSelectedSensorPosition() / ColorConstants.kEncoderCPR / ColorConstants.kTotalMechanicalAdvantage;
  }

  public WPI_TalonSRX getMotor(){
    return m_colorMotor;
  }

  public String getGameData(){
    String gameData = DriverStation.getInstance().getGameSpecificMessage();

    if(gameData.length() > 0){
      switch(gameData){
        case "B" : return "R";
        case "G" : return "Y";
        case "R" : return "B";
        case "Y" : return "G";
        default : return "X";
      }
    } else{
      return "X";
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
