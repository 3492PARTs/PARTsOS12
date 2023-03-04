// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PARTSlib2023.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import PARTSlib2023.PARTS.frc.Utils.dataHolders.PIDValues;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class angleTurn extends CommandBase {
  /** Creates a new angleTurn. */
  beanieDriveTrain driveTrain;
  double [] pidValues;
  PIDController PIDController;
  double angle;
  double initAngle;

  public angleTurn(beanieDriveTrain driveTrain, PIDValues turningValues, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.pidValues = turningValues.getPIDValues();
    this.angle = angle;
    this.PIDController = new PIDController(pidValues[0], pidValues[1], pidValues[2]);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PIDController.setSetpoint(angle);
    initAngle = driveTrain.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(PIDController.calculate(driveTrain.getAngle() - initAngle), -.1, .1);
    driveTrain.moveArcade(0.0 , output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PIDController.atSetpoint() && (driveTrain.getTurningRate() < 1);
  }
}
