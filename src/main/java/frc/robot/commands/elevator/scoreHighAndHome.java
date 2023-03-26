// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Gripper.runGripper;
import frc.robot.commands.extender.linearTrapezoid;
import frc.robot.commands.extender.profiledExtend;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreHighAndHome extends SequentialCommandGroup {
  /** Creates a new scoreHighAndHome. */
  public scoreHighAndHome() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new pivotTrapezoid(90),new linearTrapezoid(28).alongWith(new runGripper(-1).withTimeout(.6).beforeStarting(new WaitCommand(3.8))), new profiledExtend(0), new InstantCommand(() ->Elevator.getInstance().setPivotSpeed(0), Elevator.getInstance()));
  }
}
