// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.commands;

import java.util.concurrent.Callable;

import com.PARTS.frc.Utils.Controls.beanieController;
import com.PARTS.frc.Utils.Interfaces.beanieDriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class joystickDrive extends CommandBase {
    beanieDriveTrain bDriveTrain;
    beanieController controller;
    Callable<Double> leftStick;
    Callable<Double> rightStick;
  /** Creates a new joystickDrive. */

  /**
   * 
   * @param bDriveTrain the drivetrain object
   * @param leftStick a function that gives the value of the joystick axis desired to control forward back rotation
   * @param rightStick a function that gives the value of the joystick axis that controlls rotation
   */
  public joystickDrive(beanieDriveTrain bDriveTrain, Callable<Double> leftStick, Callable<Double> rightStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(bDriveTrain);
    this.bDriveTrain = bDriveTrain;
    this.leftStick = leftStick;
    this.rightStick = rightStick;

  }

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    try {
      bDriveTrain.moveArcade(rightStick.call(), leftStick.call());
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bDriveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
