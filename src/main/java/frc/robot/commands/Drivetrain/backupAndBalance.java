// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import PARTSlib2023.PARTS.frc.Utils.dataHolders.PIDValues;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.driveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class backupAndBalance extends SequentialCommandGroup {
  /** Creates a new backupAndBalance. */
  public backupAndBalance() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(3.75, .1, 0), Units.inchesToMeters(-48)).withTimeout(5), new autoLevel());
  }
}
