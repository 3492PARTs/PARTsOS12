// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class holdGripper extends CommandBase {
  /** Creates a new holdGripper. */
  public holdGripper() {
    addRequirements(Gripper.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Gripper.getInstance().hasCube()){
      Gripper.getInstance().runCurrent(10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Gripper.getInstance().runCurrent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
