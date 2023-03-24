// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.autoLevelNoPID;
import frc.robot.commands.Drivetrain.backupAndBalance;
import frc.robot.commands.Gripper.runGripper;
import frc.robot.commands.elevator.raiseArmAndDrop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class backupScoreLowBalance extends SequentialCommandGroup {
  /** Creates a new backupScoreLowBalance. */
  public backupScoreLowBalance() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new raiseArmAndDrop().andThen(new runGripper(-1).withTimeout(.5)).andThen(new autoLevelNoPID()));
  }
}
