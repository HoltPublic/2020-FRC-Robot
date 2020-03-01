/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.colorsensor.ColorControl;
//import frc.robot.commands.colorsensor.RotationControl;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.DriveBackDistance;
import frc.robot.commands.drive.DriveForwardDistance;
import frc.robot.commands.drive.FlipDrive;
import frc.robot.commands.drive.HalveDriveSpeed;
import frc.robot.commands.drive.TurnAngleRight;
import frc.robot.commands.intake.SpitOut;
import frc.robot.commands.intake.SuckIn;
import frc.robot.commands.leds.AutonLights;
import frc.robot.commands.leds.TeleOPLights;
import frc.robot.commands.lift.LightSaberUp;
import frc.robot.commands.lift.LightSaborDown;
import frc.robot.commands.lift.LowerTheBoi;
import frc.robot.commands.lift.RaiseTheBoi;
import frc.robot.commands.pneumatics.BallPistion;
import frc.robot.commands.pneumatics.RaiseColorMotor;
//import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Underglow;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  //private final ColorSensor m_color = new ColorSensor();
  private final Underglow m_glow = new Underglow();
  private final Pneumatics m_blow = new Pneumatics();
  private final Intake m_intake = new Intake();
  private final Lift m_lift = new Lift();

  // Trajectory stuff
  private final TrajectoryConfig m_config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));

  // Different types of auto commands
  private final Command m_driveDistanceAuto = new DriveForwardDistance(AutoConstants.kDriveGetOffLineInches,
      AutoConstants.kDriveSpeed, m_drive);
  private final Command m_driveToDump = new SequentialCommandGroup(
      new DriveBackDistance(AutoConstants.kDriveToDumpInches, AutoConstants.kDriveSpeed, m_drive),
      new BallPistion(m_blow).withTimeout(5));
  private final Command m_dumpInBuddy = new SequentialCommandGroup(new WaitCommand(AutoConstants.kDumpToBuddySeconds),
      new SpitOut(m_intake).withTimeout(3));
  private final Command m_comeMySon = new SequentialCommandGroup(new DriveForwardDistance(120, .5, m_drive),
      new TurnAngleRight(180, .5, m_drive), new DriveForwardDistance(120, .5, m_drive));
  private final Command m_driveToTrench = getRamCommand("DriveToTrench.wpilib.json");

  // A chooser for auto commands
  private final SendableChooser<Command> m_auto = new SendableChooser<>();

  // Controllers
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OIConstants.kOperatorContollerPort);

  // Cameras
  private final UsbCamera m_frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
  private final UsbCamera m_backCamera = CameraServer.getInstance().startAutomaticCapture(1);

  // Tab for Shuffleboard
  private final ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main Tab");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // More Trajectory stuff
    m_config.setKinematics(m_drive.getKinematics());

    // Add Commands to the auton command chooser
    m_auto.setDefaultOption("Distance Drive Auto", m_driveDistanceAuto);
    m_auto.addOption("Drive and Dump", m_driveToDump);
    m_auto.addOption("Dump in Buddy", m_dumpInBuddy);
    m_auto.addOption("My Son", m_comeMySon);
    m_auto.addOption("Drive to Trench", m_driveToTrench);
    m_auto.addOption("Do Nothing", new WaitCommand(15));


    // Sets the default commands
    m_drive.setDefaultCommand(new ArcadeDrive(() -> -m_driverController.getY(GenericHID.Hand.kLeft), () -> m_driverController.getX(GenericHID.Hand.kLeft), m_drive));
    m_glow.setDefaultCommand(new TeleOPLights(m_glow));

    // Settings for the cameras
    //m_frontCamera.setResolution(50, 50);

    // Put all the stuff on the Dashboard
    m_mainTab.add("Auto Chooser", m_auto).withSize(2, 1).withPosition(0, 0);
    m_mainTab.add("Front Camera", m_frontCamera).withSize(3, 3).withPosition(2, 0);
    m_mainTab.add("Back Camera", m_backCamera).withSize(3, 3).withPosition(5, 0);
    //m_mainTab.add("Color Detected", m_color.getColor()).withSize(2, 1).withPosition(0, 2);
    //m_mainTab.add("Drive", m_drive.getDrive()).withSize(3, 2).withPosition(5, 0);
    m_mainTab.add("Rotations", m_lift.getRotation());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Controls */
    // While holding the Shoulder Button drive slow
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenHeld(new HalveDriveSpeed(m_drive));
    // While holding the other Shoulder Button it flips the front
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenHeld(new FlipDrive(() -> -m_driverController.getY(GenericHID.Hand.kLeft),
            () -> m_driverController.getX(GenericHID.Hand.kLeft), m_drive));
    // When holding button the Winch will spin backwards
    new JoystickButton(m_driverController, Button.kBack.value).whenHeld(new LowerTheBoi(m_lift));

    /* Operator Controls */
    // When the trigger is pressed block the balls
    new JoystickButton(m_operatorController, 1).whenHeld(new BallPistion(m_blow));
    // When button is pressed starts the intake
    new JoystickButton(m_operatorController, 5).whenHeld(new SuckIn(m_intake));
    // When button is pressed reverses the intake
    new JoystickButton(m_operatorController, 6).whenHeld(new SpitOut(m_intake));
    // When button is pressed lift the robot
    new JoystickButton(m_operatorController, 16).whenHeld(new RaiseTheBoi(m_lift));
    // When button is pressed the color wheel pistion will toggle
    new JoystickButton(m_operatorController, 2).whenPressed(new RaiseColorMotor(m_blow));
    // When button is pressed the color wheel will do rotation control
    //new JoystickButton(m_operatorController, 13).whileHeld(new RotationControl(m_color));
    // When button is pressed the color wheel will do color control
    //new JoystickButton(m_operatorController, 14).whileHeld(new ColorControl(m_color));
    // When button is pressed the telescope will go up
    new JoystickButton(m_operatorController, 9).whenPressed(new LightSaberUp(m_lift));
    // WHen button is pressed the telescope will go down
    new JoystickButton(m_operatorController, 8).whenPressed(new LightSaborDown(m_lift));
  }

  public Command getAutonLights(){
    return new AutonLights(m_glow);
  }
  
  public Command getRamCommand(String trajectoryJSON){
    trajectoryJSON = "output/" + trajectoryJSON;
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return new RamseteCommand(
        trajectory,
        m_drive::getPose,
        new RamseteController(2.0, 0.7),
        m_drive.getFeedforward(),
        m_drive.getKinematics(),
        m_drive::getSpeeds,
        m_drive.getLeftPIDController(),
        m_drive.getRightPIDController(),
        m_drive::setOutput,
        m_drive
      );
    } catch(IOException ex){
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      return new WaitCommand(15);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return m_auto.getSelected();
  }
}
