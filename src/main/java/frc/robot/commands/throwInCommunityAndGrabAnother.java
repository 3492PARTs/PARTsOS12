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
import frc.robot.commands.Gripper.runGripper;
import frc.robot.commands.extender.linearTrapezoid;
import frc.robot.subsystems.driveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class throwInCommunityAndGrabAnother extends SequentialCommandGroup {
  /** Creates a new throwInCommunityAndGrabAnother. */
  public throwInCommunityAndGrabAnother() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(3.75, .1, 0), Units.inchesToMeters(-12)).withTimeout(1),new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(3.75, .1, 0), Units.inchesToMeters(50)).withTimeout(4), new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(3.75, .1, 0), Units.inchesToMeters(70)).alongWith(new linearTrapezoid(26)).alongWith(new intakeGripper(1)),new intakeGripper(1).withTimeout(3));
  }
}
