// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PARTSlib2023.PARTS.frc.Utils.dataHolders.PIDValues;
import PARTSlib2023.PARTS.frc.commands.PIDTurn;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.PIDdrive;
import frc.robot.commands.Gripper.intakeGripper;
import frc.robot.commands.elevator.pivotTrapezoid;
import frc.robot.commands.elevator.raiseArmAndDrop;
import frc.robot.commands.extender.linearTrapezoid;
import frc.robot.commands.extender.stopExtender;
import frc.robot.subsystems.driveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class backupAndGrabTurnLeft extends SequentialCommandGroup {
  /** Creates a new backupAndGrabTurnLeft. */
  public backupAndGrabTurnLeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new raiseArmAndDrop().andThen(new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(2.55, .1, 0), Units.inchesToMeters(-191)).withTimeout(4)).andThen(new PIDTurn(driveTrain.geDriveTrain(), new PIDValues(0.001, 0.0005, 0), 171).withTimeout(2.5)), (new linearTrapezoid(16).andThen(new stopExtender())), new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(2.55, .1, 0), Units.inchesToMeters(38)).withTimeout(4).raceWith(new intakeGripper(1)));
  }
}
