// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain;

public class autoLevel extends CommandBase {
  /** Creates a new autoLevel. */
  PIDController angController = new PIDController(.00625, 0.003, 0);
  public autoLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain.getDriveTrainInstance());
    angController.setIntegratorRange(-10, 10);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(angController.calculate(driveTrain.getDriveTrainInstance().getPitch()) , -.2 , .2);
    driveTrain.getDriveTrainInstance().move(-output, -output);

    System.out.println(-driveTrain.getDriveTrainInstance().getPitch());
    System.out.println();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.getDriveTrainInstance().move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(driveTrain.getDriveTrainInstance().getPitch()) < 1) && Math.abs(driveTrain.getDriveTrainInstance().getRightVelocity()) < .05 && driveTrain.getDriveTrainInstance().getVelocityGyro() < .1;
  }
}
