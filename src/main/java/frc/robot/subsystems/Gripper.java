// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Gripper.holdGripper;

public class Gripper extends SubsystemBase {
  private TalonSRX leftGripper;
  private TalonSRX BottomGripper;
  private TalonSRX rightGripper;
  private static Gripper gripper = new Gripper();
  private boolean hasGamePiece = true;
  DigitalInput photoEye = new DigitalInput(0);


  /** Creates a new Gripper. */
  public Gripper() {

    leftGripper = new TalonSRX(13);
    rightGripper = new TalonSRX(29);
    BottomGripper = new TalonSRX(27);

    Shuffleboard.getTab(Constants.debugTab).addNumber("leftGripperCurrent", gripperLeftCurrentSupplier());
    Shuffleboard.getTab(Constants.debugTab).addNumber("rightGripperCurrent", gripperRightCurrentSupplier());
    SmartDashboard.putBoolean("HAS CUBE", hasCubeSupplier().getAsBoolean());
    
    // setDefaultCommand(new holdGripper());

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
    leftGripper.set(ControlMode.PercentOutput, -speed);
    rightGripper.set(ControlMode.PercentOutput, -speed);
    //BottomGripper.set(ControlMode.PercentOutput, speed);
  }

  public DoubleSupplier gripperLeftCurrentSupplier() {
    DoubleSupplier s = () -> leftGripper.getSupplyCurrent();
    return s;
  }

  public DoubleSupplier gripperRightCurrentSupplier() {
    DoubleSupplier s = () -> rightGripper.getSupplyCurrent();
    return s;
  }

  public void runCurrent(double current){
    leftGripper.set(ControlMode.Current, current);
    rightGripper.set(ControlMode.Current, current);
    BottomGripper.set(ControlMode.Current, current);
  }

  public Boolean hasCube() {
    return photoEye.get();
  }

  public BooleanSupplier hasCubeSupplier(){
    BooleanSupplier s = () -> 
      hasCube();    
    return s;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("HAS CUBE", hasCubeSupplier().getAsBoolean());
    SmartDashboard.putBoolean("Sensor 0", photoEye.get());

    // This method will be called once per scheduler run
  }

  
}
