// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Gripper.holdGripper;

public class Gripper extends SubsystemBase {
  private TalonSRX leftGripper;
  private TalonSRX rightGripper;
  private static Gripper gripper = new Gripper();
  private boolean hasGamePiece = true;

  /** Creates a new Gripper. */
  public Gripper() {

    leftGripper = new TalonSRX(13);
    rightGripper = new TalonSRX(29);

    Shuffleboard.getTab(Constants.debugTab).add("leftGripperCurrent", gripperLeftCurrentSupplier());
    Shuffleboard.getTab(Constants.debugTab).add("rightGripperCurrent", gripperRightCurrentSupplier());
    
    setDefaultCommand(new holdGripper());

  }

  public static Gripper getInstance() {
    return gripper;
  }

  public boolean hasGamePiece(){
    return hasGamePiece;
  }

  public void setGamePiece(boolean hasGamePiece){
    this.hasGamePiece = hasGamePiece;
  }

  public void runGripper(double speed) {
    leftGripper.set(ControlMode.PercentOutput, speed);
    rightGripper.set(ControlMode.PercentOutput, speed);
  }

  public DoubleSupplier gripperLeftCurrentSupplier() {
    DoubleSupplier s = () -> leftGripper.getStatorCurrent();
    return s;
  }

  public DoubleSupplier gripperRightCurrentSupplier() {
    DoubleSupplier s = () -> rightGripper.getStatorCurrent();
    return s;
  }

  public void runCurrent(double current){
    leftGripper.set(ControlMode.Current, current);
    rightGripper.set(ControlMode.Current, current);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
