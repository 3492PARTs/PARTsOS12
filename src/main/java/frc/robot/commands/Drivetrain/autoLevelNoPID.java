// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain;

public class autoLevelNoPID extends CommandBase {
  /** Creates a new autoLevelNoPID. */
  public autoLevelNoPID() {
    addRequirements(driveTrain.getDriveTrainInstance());

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveTrain.geDriveTrain().getPitch() > 2){
      driveTrain.getDriveTrainInstance().move(.25, .25);
    }
    
    else if(driveTrain.geDriveTrain().getPitch() < -2){
      driveTrain.getDriveTrainInstance().move(-.25, -.25);
    }

    System.out.println("running auto level with no pid" + driveTrain.getDriveTrainInstance().getPitch());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.getDriveTrainInstance().move(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(driveTrain.getDriveTrainInstance().getPitch()) < 2) && Math.abs(driveTrain.getDriveTrainInstance().getRightVelocity()) < .05 && driveTrain.getDriveTrainInstance().getVelocityGyro() < .1;
  }
}
