// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class runElevatorTeleop extends CommandBase {
  /** Creates a new runGripperTeleop. */
  CommandXboxController opCommandXboxController;
  public runElevatorTeleop(CommandXboxController opController) {
    addRequirements(Elevator.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
    this.opCommandXboxController = opController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(Elevator.getInstance().calcHoldingVoltage());
    if(Math.abs(opCommandXboxController.getLeftY()) > .1){
      Elevator.getInstance().setPivotSpeed(-.5 * opCommandXboxController.getLeftY());
    }

    else Elevator.getInstance().driveMotorVolts(Elevator.getInstance().calcOutputVoltage(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
