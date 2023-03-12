// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.linearExtension;

public class linearController extends CommandBase {
  /** Creates a new linearController. */
  double direction;
  public linearController(double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.direction = direction;
    addRequirements(linearExtension.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("going in or out");
    linearExtension.getInstance().setLinearSpeed(direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linearExtension.getInstance().setLinearSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
