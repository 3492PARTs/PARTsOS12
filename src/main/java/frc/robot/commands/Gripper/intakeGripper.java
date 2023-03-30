// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class intakeGripper extends CommandBase {
  /** Creates a new runGripper. */
  double direction;

  public intakeGripper(double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Gripper.getInstance());
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Gripper.getInstance().hasCube() && direction > 0) {
      Gripper.getInstance().runGripper(0.2);
    }
    else {
      Gripper.getInstance().runGripper(1 * direction);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Gripper.getInstance().runGripper(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Gripper.getInstance().hasCube();
  }
}
