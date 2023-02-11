// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevatormotors;


public class Disttance extends CommandBase {
  PIDController pid = new PIDController(0, 0, 0);
  elevatormotors m_Elevatormotors = elevatormotors.getElevatormotorsInstance();
  public double setPoint;
    public Disttance(double setPoint) {
      addRequirements(m_Elevatormotors);
      
    }
  
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    double output;
    output = (pid.calculate(elevatormotors.getElevatormotorsInstance().getPosition(), setPoint));
    elevatormotors.getElevatormotorsInstance().extendarm(output);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
