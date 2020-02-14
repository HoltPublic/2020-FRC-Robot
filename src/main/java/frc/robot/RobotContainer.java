/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveBackDistance;
import frc.robot.commands.drive.DriveForwardDistance;
import frc.robot.commands.drive.FlipDrive;
import frc.robot.commands.drive.HalveDriveSpeed;
import frc.robot.commands.drive.StartMusic;
import frc.robot.commands.drive.TurnAngleRight;
import frc.robot.commands.intake.SpitOut;
import frc.robot.commands.intake.SuckIn;
import frc.robot.commands.leds.AutonLights;
import frc.robot.commands.leds.TeleOPLights;
import frc.robot.commands.lift.LightSaberUp;
import frc.robot.commands.lift.LowerTheBoi;
import frc.robot.commands.lift.RaiseTheBoi;
import frc.robot.commands.pneumatics.BallPistion;
import frc.robot.commands.colorsensor.GetColorName;

import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Underglow;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ColorSensor m_color = new ColorSensor();
  private final Underglow m_glow = new Underglow();
  private final Pneumatics m_blow = new Pneumatics();
  private final Intake m_intake = new Intake();
  private final Lift m_lift = new Lift();

  // Different types of auto commands
  private final Command m_driveDistanceAuto = new DriveForwardDistance(AutoConstants.kDriveGetOffLineInches, AutoConstants.kDriveSpeed, m_drive);
  private final Command m_driveToDump = new SequentialCommandGroup(new DriveBackDistance(AutoConstants.kDriveToDumpInches, AutoConstants.kDriveSpeed, m_drive), new SuckIn(m_intake).withTimeout(5));
  private final Command m_dumpInBuddy = new SequentialCommandGroup(new WaitCommand(AutoConstants.kDumpToBuddySeconds), new SpitOut(m_intake).withTimeout(5));
  private final Command m_comeMySon = new SequentialCommandGroup(new DriveForwardDistance(120, .5, m_drive), new TurnAngleRight(180, .5, m_drive), new DriveForwardDistance(120, .5, m_drive));

  // A chooser for auto commands
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // A chooser for the song
  private final SendableChooser<String> m_song = new SendableChooser<>();

  // Controllers
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OIConstants.kOperatorContollerPort);

  // Cameras
  private final UsbCamera m_camera = CameraServer.getInstance().startAutomaticCapture();

  // Tab for Shuffleboard
  private final ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main Tab");

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Sets the default commands
    m_drive.setDefaultCommand(new DefaultDrive(() -> -m_driverController.getY(GenericHID.Hand.kLeft), () -> m_driverController.getX(GenericHID.Hand.kLeft), m_drive));
    m_color.setDefaultCommand(new GetColorName(m_color));
    m_glow.setDefaultCommand(new TeleOPLights(m_glow));

    // Add Commands to the auton command chooser
    m_chooser.setDefaultOption("Distance Drive Auto", m_driveDistanceAuto);
    m_chooser.addOption("Drive and Dump", m_driveToDump);
    m_chooser.addOption("Dump in Buddy", m_dumpInBuddy);
    m_chooser.addOption("My Son", m_comeMySon);
    m_chooser.addOption("Do Nothing", new WaitCommand(15));

    // The songs you can choose
    m_song.setDefaultOption("Megalovania", "mega.chrp");
    m_song.addOption("Turret Song", "turret.chrp");
    m_song.addOption("Renai Circulation", "renai.chrp");
    m_song.addOption("Servant of Evil", "servant.chrp");
    m_song.addOption("Flamingo", "flamingo.chrp");
    m_song.addOption("Star Spangled Banner", "star.chrp");

    // Settings for the cameras
    m_camera.setResolution(720, 480);

    // Put the choosers and cameras on the dashboard
    m_mainTab.add(m_chooser).withSize(2, 1).withPosition(0, 0);
    m_mainTab.add(m_song).withSize(2, 1).withPosition(0, 1);
    m_mainTab.add(m_camera).withSize(3, 3).withPosition(2, 0);

    Shuffleboard.getTab("Main Tab").add("Encoder", m_lift.m_telescope.getSelectedSensorPosition());

    m_mainTab.add(m_drive.getDrive()).withSize(3, 2).withPosition(5, 0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*  Driver Controls */
    // While holding the Shoulder Button drive slow
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenHeld(new HalveDriveSpeed(m_drive));
    // While holding the other Shoulder Button it flips the front
    new JoystickButton(m_driverController, Button.kBumperRight.value).whenHeld(new FlipDrive(() -> -m_driverController.getY(GenericHID.Hand.kLeft), () -> m_driverController.getX(GenericHID.Hand.kLeft), m_drive));
    // When button is pressed music will play from falcons
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(new StartMusic(m_song.getSelected(), m_drive));
    // When holding button the Winch will spin backwards
    new JoystickButton(m_driverController, Button.kBack.value).whenHeld(new LowerTheBoi(m_lift));

    new JoystickButton(m_driverController, Button.kA.value).whenPressed(new LightSaberUp(m_lift));
    
    /*  Operator Controls */
    // When the trigger is pressed block the balls
    new JoystickButton(m_operatorController, 1).whenHeld(new BallPistion(m_blow));
    // When button is pressed starts the intake
    new JoystickButton(m_operatorController, 6).whenHeld(new SuckIn(m_intake));
    // When button is pressed reverses the intake
    new JoystickButton(m_operatorController, 5).whenHeld(new SpitOut(m_intake));
    // When button is pressed lift the robot
    new JoystickButton(m_operatorController, 16).whenHeld(new RaiseTheBoi(m_lift));
  }

  public Command getAutonLights(){
    return new AutonLights(m_glow);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));

    config.setKinematics(m_drive.getKinematics());
    
    RamseteCommand command = new RamseteCommand(
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

    
    return m_chooser.getSelected();
  }
}