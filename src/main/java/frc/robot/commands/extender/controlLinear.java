// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.linearExtension;

public class controlLinear extends CommandBase {
  /** Creates a new controlLinear. */
  double extension;
  public controlLinear(double extension) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    this.extension = extension;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linearExtension.getInstance().setSetPointLinear(extension);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linearExtension.getInstance().setLinearSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(extension - linearExtension.getInstance().getExtension()) < Units.inchesToMeters(1)) && linearExtension.getInstance().getExtensionRate() < .05; 
  }
}
