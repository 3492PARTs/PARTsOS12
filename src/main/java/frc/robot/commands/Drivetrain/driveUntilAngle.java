// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain;

public class driveUntilAngle extends CommandBase {
  /** Creates a new driveUntilAngle. */
  boolean reverse = false;
  public driveUntilAngle() {
    addRequirements(driveTrain.getDriveTrainInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public driveUntilAngle(boolean reverse) {
    addRequirements(driveTrain.getDriveTrainInstance());
    // Use addRequirements() here to declare subsystem dependencies.
    this.reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(reverse){
      driveTrain.getDriveTrainInstance().move(.37,.37); // go forward
    }
    else{
    driveTrain.getDriveTrainInstance().move(-.37, -.37);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.getDriveTrainInstance().move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!reverse){
      return (driveTrain.getDriveTrainInstance().getPitch() < 4) && driveTrain.getDriveTrainInstance().getPitch() > 0;
    }
    if(reverse){
      return (driveTrain.getDriveTrainInstance().getPitch() > 4) && driveTrain.getDriveTrainInstance().getPitch() > 0;
    }
    else return (driveTrain.getDriveTrainInstance().getPitch() < 4) && driveTrain.getDriveTrainInstance().getPitch() > 0;
  }
}
