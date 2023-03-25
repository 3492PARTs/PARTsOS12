// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.linearExtension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class profiledExtend extends ProfiledPIDCommand {
  /** Creates a new profiledExtend. */
  public profiledExtend(double extensionInches) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0,
            0,
            0,
            // The motion profile constraints
            linearExtension.getInstance().getConstraints()
            ),
        // This should return the measurement
        () -> linearExtension.getInstance().getExtension(),
        // This should return the goal (can also be a constant)
        new TrapezoidProfile.State(Units.inchesToMeters(extensionInches), 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          System.out.println(setpoint.velocity + "extending like a profile");
          double volts = linearExtension.getInstance().calcOutputVoltage(setpoint.velocity);
          linearExtension.getInstance().setLinearVoltage(volts);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(linearExtension.getInstance());
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Units.inchesToMeters(.1));
  }

  // Returns true when the command should end.
  @Override
  public void end(boolean interrupted) {
      super.end(interrupted);
      linearExtension.getInstance().setLinearVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
