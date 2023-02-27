// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorState;

public class linearExtender extends SubsystemBase {
  double linearGearRatio = 4.0;
  CANSparkMax linearMotor;
  SparkMaxPIDController linear1Controller;
  double wheelCircumference = 1;
  // state , extension distance
  public static HashMap<elevatorState, Double> extenderStateMap;

  static linearExtender extender = new linearExtender();
  /** Creates a new linearExtender. */
  public linearExtender() {
    linearMotor = new CANSparkMax(9, MotorType.kBrushless);
    linearMotor.setSmartCurrentLimit(15, 15);
    linear1Controller = linearMotor.getPIDController();
    linear1Controller = linearMotor.getPIDController();
    linearMotor.setIdleMode(IdleMode.kBrake);


    Shuffleboard.getTab("debug").addNumber("arm Linear Extension", getExtensionDistanceSupplier());
    Shuffleboard.getTab("debug").addNumber("arm linear velocity", getExtensionRateSupplier());


  }


  public static linearExtender getInstance(){
    return extender;
  }

  public DoubleSupplier getExtensionDistanceSupplier() {
    DoubleSupplier s = () -> getExtension();
    return s;
  }

  public DoubleSupplier getExtensionRateSupplier() {
    DoubleSupplier s = () -> getExtensionRate();
    return s;
  }

  public double getExtension() {
    return wheelCircumference * (linearMotor.getEncoder().getPosition() / linearGearRatio);
  }

  public double getExtensionRate() {
    return wheelCircumference * (linearMotor.getEncoder().getVelocity() / linearGearRatio);
  } 
  public void setLinearSpeed(double speed) {
    linearMotor.set(speed);
  }
  public void setSetPointLinear(double extensionMeters) {
    linear1Controller.setReference(extensionMeters * linearGearRatio, ControlType.kPosition);
  }

  
  public void zeroPivotEncoder() {
    linearMotor.getEncoder().setPosition(0);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
