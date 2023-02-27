// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class holdPosition extends CommandBase {
  /** Creates a new holdPosition. */
  PIDController velocityHolder;

  public holdPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    velocityHolder = new PIDController(0, 0, 0);
    velocityHolder.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double output = MathUtil.clamp(velocityHolder.calculate(Elevator.getInstance().getRotationRate()), -.05, .05);

    Elevator.getInstance().setPivotSpeed(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Elevator.getInstance().setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
