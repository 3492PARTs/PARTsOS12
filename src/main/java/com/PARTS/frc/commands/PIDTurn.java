// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.commands;


import com.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import com.PARTS.frc.Utils.dataHolders.PIDValues;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class PIDTurn extends CommandBase {
  /** Creates a new PIDDrive. */
  double initAngle;
  double setPoint;
  beanieDriveTrain driveTrain;
  double[] pidValues;
  PIDController PIDController;
  /**
   * 
   * @param driveTrain a drivetrain implementing the proper interface
   * @param turningValues tested pid values for turning you can likely use the ones from the limelight
   */
  public PIDTurn(beanieDriveTrain driveTrain, PIDValues turningValues) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrain = driveTrain;
    this.pidValues = turningValues.getPIDValues();
    PIDController = new PIDController(pidValues[0], pidValues[1], pidValues[2]);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initAngle = driveTrain.getAngle();
    PIDController.setSetpoint(setPoint);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = PIDController.calculate(driveTrain.getAngle());

    speed = MathUtil.clamp(speed, -1, 1);

    driveTrain.move(speed, -speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PIDController.atSetpoint() && (driveTrain.getTurningRate() < .5);
  }
}
