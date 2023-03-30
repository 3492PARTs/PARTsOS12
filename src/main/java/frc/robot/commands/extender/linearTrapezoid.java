// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.linearExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class linearTrapezoid extends TrapezoidProfileCommand {
  MedianFilter finishingFilter = new MedianFilter(8);
  /** Creates a new linearTrapezoid. */
  public linearTrapezoid(double ExtensionDistanceInches) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            linearExtension.getInstance().getConstraints(),
            // Goal state
            new TrapezoidProfile.State(Units.inchesToMeters(ExtensionDistanceInches) , 0),
            // Initial state
            linearExtension.getInstance().getState()
            ),
        state -> {
          
          // Use current trajectory state here
          linearExtension.getInstance().setGoalState(state);
          double output = linearExtension.getInstance().calcOutputVoltage(linearExtension.getInstance().getGoalState().velocity);
          linearExtension.getInstance().setLinearVoltage(output);
          System.out.println("like a flat trapezoid" + state.velocity);

        },
        linearExtension.getInstance());

  }
}
