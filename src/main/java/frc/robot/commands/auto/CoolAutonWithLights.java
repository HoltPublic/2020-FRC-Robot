/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.leds.AutonLights;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Underglow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CoolAutonWithLights extends ParallelCommandGroup {
  /**
   * Creates a new CoolAutonWithLights.
   */
  public CoolAutonWithLights(Underglow glow, DriveSubsystem drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(new AutonLights(glow), new DriveSpinDrive(drive));
  }
}
