// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pivotTrapezoid extends TrapezoidProfileCommand {
  /** Creates a new pivotTrapezoid. */
  public pivotTrapezoid(double goalAngle) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            Elevator.getInstance().getConstraints(),
            // Goal state
            new TrapezoidProfile.State(Math.toRadians(goalAngle), 0),
            // Initial state
            Elevator.getInstance().getState()
            ),
        state -> {
          // Use current trajectory state here
            Elevator.getInstance().setGoalState(state);
            double output = Elevator.getInstance().calcOutputVoltage(Elevator.getInstance().getGoalState().velocity);
            Elevator.getInstance().driveMotorVolts(output);
            System.out.println(Elevator.getInstance().getGoalState().velocity);
            System.out.println("pivoting like a trapezoid");
        },
        Elevator.getInstance());
  }
}
