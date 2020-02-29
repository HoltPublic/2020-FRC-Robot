/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterInches = 6;
        public static final double kTrackWidthInches = 21.75;
        public static final double kGearRatio = (50 / 14) * (48 / 16);
        public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
        public static final double kMetersPerTick = Units.inchesToMeters(kWheelCircumferenceInches) / (kEncoderCPR * kGearRatio);

        //TODO: Change theses for our robot
        public static final double kS = 0.268;
        public static final double kV = 1.89;
        public static final double kA = 0.243;

        public static final double kP = 9.95;
    }

    public static final class AutoConstants{
        public static final double kDriveSpeed = .5;
        public static final double kDriveDistanceSeconds = 4;
        public static final double kDriveGetOffLineInches = 120;
        public static final double kDriveToDumpInches = 110;
        public static final double kDumpToBuddySeconds = 3;

        //TODO: Change theses for our robot
        public static final double kMaxSpeedMetersPerSec = 2;
        public static final double kMaxAccelerationMetersPerSec = 2;
    }

    public static final class BlinkinConstants{
        public static final int kBlinkinPWM = 0;
    }

    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorContollerPort = 1;
    }

    public static final class PneumaticsConstants{
        public static final int kBallStopperForward = 0;
        public static final int kBallStopperBackward = 1;
        public static final int kColorMotor = 2;
    }

    public static final class LiftConstants{
        public static final int kClimbMotor = 5;
        public static final int kTelescopeMotor = 6;

        public static final int kEncoderCPR = 2048;
        public static final double kTelescopeSpeedUp = 0.2;
        public static final double kTelescopeSpeedDown = 0.1;
        public static final double kRotationsToGoUp = 12;
    }

    public static final class IntakeConstants{
        public static final int kBeltMotor = 7;
    }

    public static final class ColorConstants{
        public static final int kColorMotor = 8;
        public static final int kEncoderCPR = 12;
        public static final double kColorWheelDiameterInches = 32;
        public static final double kSpinnerWheelDiameterInches = 3;
        public static final double kTotalMechanicalAdvantage = kColorWheelDiameterInches / kSpinnerWheelDiameterInches;
    }
}
