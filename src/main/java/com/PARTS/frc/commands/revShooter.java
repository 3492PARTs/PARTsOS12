// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.commands;


import com.PARTS.frc.Utils.Interfaces.beanieShooter;
import com.ctre.phoenix.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class revShooter extends CommandBase {

  BangBangController bController;
  /** Creates a new revShooter. */
  double velocity, maxOutput;
  beanieShooter bShooter;
  /**
   * 
   * @param velocity the velocity you want to acheive
   * @param maxOutput the maximum percent output
   * @param bShooter a shooter implementing the beanie shooter interface.
   * @implNote this command does not stop on its own termiate it with a command group or with some other method
   */
  public revShooter(double velocity, double maxOutput, beanieShooter bShooter) {
    this.velocity = velocity;
    this.maxOutput = maxOutput;
    this.bShooter = bShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    bController = new BangBangController();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(bController.calculate(bShooter.getRPM(), velocity), -maxOutput, maxOutput);
    bShooter.shooterOutput(output);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bShooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
