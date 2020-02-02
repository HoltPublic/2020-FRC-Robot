/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;

import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.HalveDriveSpeed;
import frc.robot.commands.intake.SpitOut;
import frc.robot.commands.intake.SuckIn;
import frc.robot.commands.auto.CoolAutonWithLights;
import frc.robot.commands.auto.SimpleDriveWithLights;
import frc.robot.commands.leds.AutonLights;
import frc.robot.commands.leds.TeleOPLights;
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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ColorSensor m_color = new ColorSensor();
  private final Underglow m_glow = new Underglow();
  private final Pneumatics m_blow = new Pneumatics();
  private final Intake m_intake = new Intake();
  private final Lift m_lift = new Lift();

  // the stuff for auton
  // Auto that just drives for a few seconds and stops
  private final Command m_driveAuto = new SimpleDriveWithLights(m_glow, m_robotDrive);
  private final Command m_complexAuto = new CoolAutonWithLights(m_glow, m_robotDrive);

  // Auto that does nothing
  private final Command m_nothingAuto = new AutonLights(m_glow);

  // A chooser for auto commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_operatorController = new Joystick(OIConstants.kOperatorContollerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Sets the default commands
    m_robotDrive.setDefaultCommand(new DefaultDrive(m_robotDrive,() -> -m_driverController.getY(GenericHID.Hand.kLeft),() -> m_driverController.getX(GenericHID.Hand.kLeft)));
    m_color.setDefaultCommand(new GetColorName(m_color));
    m_glow.setDefaultCommand(new TeleOPLights(m_glow));

    // Add Commands to the auton command chooser
    m_chooser.setDefaultOption("Drive Auto", m_driveAuto);
    m_chooser.addOption("Cool boi", m_complexAuto);
    m_chooser.addOption("Do Nothing", m_nothingAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // While holding the Shoulder Button drive slow
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenHeld(new HalveDriveSpeed(m_robotDrive));
    
    new JoystickButton(m_operatorController, 0).whenHeld(new BallPistion(m_blow));
    new JoystickButton(m_operatorController, 5).whenHeld(new SuckIn(m_intake));
    new JoystickButton(m_operatorController, 4).whenHeld(new SpitOut(m_intake));
    new JoystickButton(m_operatorController, 15).whenHeld(new RaiseTheBoi(m_lift));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}