// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class controlPivot extends CommandBase {
  /** Creates a new controlPivot. */
  double angle;
  public controlPivot(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Elevator.getInstance().setSetPointPivot(angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().setPivotSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(angle - Elevator.getInstance().getAngle()) < 3) && Elevator.getInstance().getRotationRate() < 30;
  }
}
