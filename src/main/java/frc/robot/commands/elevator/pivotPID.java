// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class pivotPID extends CommandBase {
  /** Creates a new pivotPID. */
  double goal;
  PIDController pivotController = new PIDController(.01, 0, 0);
  
  public pivotPID(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    goal = Math.toRadians(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double FeedForward = Elevator.getInstance().calcHoldingVoltage(); 
    Elevator.getInstance().setPivotSpeed(FeedForward + pivotController.calculate(Math.toRadians(Elevator.getInstance().getAngle())));
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
