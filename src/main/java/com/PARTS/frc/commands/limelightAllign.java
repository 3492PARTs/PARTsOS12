// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.commands;



import com.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import com.PARTS.frc.Utils.dataHolders.PIDValues;
import com.PARTS.frc.Utils.sensors.BeanieLimeLight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class limelightAllign extends CommandBase {

  double initAngle;
  double setPoint;
  beanieDriveTrain driveTrain;
  double[] pidValues;
  BeanieLimeLight bLimeLight;
  PIDController PIDController;

  /**
   * 
   * @param driveTrain a drivetrain implementing the beanie drivetrain interface
   * @param turningValues tested pid values stored in the PIDValues class
   * @param limeLight an object of the BeanieLimeLight class
   */
  /** Creates a new limelightAllign. */
  public limelightAllign(beanieDriveTrain driveTrain, PIDValues turningValues, BeanieLimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.    
    this.driveTrain = driveTrain;
    this.pidValues = turningValues.getPIDValues();
    this.bLimeLight = limeLight;
    this.PIDController = new PIDController(pidValues[0], pidValues[1], pidValues[2]);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PIDController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(PIDController.calculate(bLimeLight.getTx()), -1, 1);
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
