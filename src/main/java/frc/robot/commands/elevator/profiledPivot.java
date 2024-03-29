// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class profiledPivot extends ProfiledPIDCommand {
  /** Creates a new profiledPivot. */
  public profiledPivot(double angle) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            1.05,
            8,
            .001,
            // The motion profile constraints
            Elevator.getInstance().getConstraints()),
            // This should return the measurement
        () -> Elevator.getInstance().getState().position,
        // This should return the goal (can also be a constant)
        new TrapezoidProfile.State(Math.toRadians(angle), 0),
        // This uses the output
        (output, setpoint) -> {

          double volts = Elevator.getInstance().calcOutputVoltage(setpoint.velocity);
          Elevator.getInstance().driveMotorVolts(volts + output);
          System.out.println(volts + output);
          // Use the output (and setpoint, if desired) here
        });
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}