// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class linearExtension extends SubsystemBase {

  double kS;
  double kV;
  double kA;

  CANSparkMax linearMotor;
  SparkMaxPIDController linear1Controller;

  double linearGearRatio = 16.0;

  double wheelCircumference = 1;

  SimpleMotorFeedforward linearFeedforward;
  TrapezoidProfile.Constraints linearConstraints;
  TrapezoidProfile.State stateToBeExecuted = new TrapezoidProfile.State(0,0);
  private static linearExtension m_elevator = new linearExtension();


  public linearExtension() {
    linearMotor = new CANSparkMax(9, MotorType.kBrushless);
    linearConstraints = new TrapezoidProfile.Constraints(Units.inchesToMeters(6),Units.inchesToMeters(3));// degrees and degrees/s


    

    linear1Controller = linearMotor.getPIDController();

    linearMotor.setSmartCurrentLimit(15, 15);

    linear1Controller = linearMotor.getPIDController();
    linearMotor.setIdleMode(IdleMode.kBrake);

    

    // pivot1Controller.setP(kP);
    // pivot1Controller.setI(kI);
    // pivot1Controller.setD(kD);
    // pivot1Controller.setIZone(kIz);
    // pivot1Controller.setFF(kFF)
    // pivot1Controller.setOutputRange(-.5, .5);

    // linear1Controller.setP(kP);
    // linear1Controller.setI(kI);
    // linear1Controller.setD(kD);
    // linear1Controller.setIZone(kIz);
    // linear1Controller.setFF(kFF);
    // linear1Controller.setOutputRange(-.1, .08);


    Shuffleboard.getTab("debug").addNumber("arm Linear Extension", getExtensionDistanceSupplier());
    Shuffleboard.getTab("debug").addNumber("arm linear velocity", getExtensionRateSupplier());

  }

  public static linearExtension getInstance() {
    return m_elevator;
  }


  public TrapezoidProfile.Constraints getConstraints(){
    return linearConstraints;
  }

  public TrapezoidProfile.State getState(){
    return new TrapezoidProfile.State(getExtension(), getExtensionRate());
  }

  public void setGoalState(TrapezoidProfile.State goalState){
    this.stateToBeExecuted = goalState;
  }

  
  public double calcOutputVoltage(double velocity) {
    double output = linearFeedforward.calculate(velocity);
    return output;
  }

  
  public TrapezoidProfile.State getGoalState (){
    return stateToBeExecuted;
  }

  public DoubleSupplier getExtensionDistanceSupplier() {
    DoubleSupplier s = () -> Units.inchesToMeters(getExtension());
    return s;
  }

  public DoubleSupplier getExtensionRateSupplier() {
    DoubleSupplier s = () -> Units.inchesToMeters(getExtensionRate());
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

  public void setLinearVoltage(double volts) {
    linearMotor.setVoltage(volts);
  }


  public void setSetPointLinear(double extensionMeters) {
    linear1Controller.setReference(extensionMeters * linearGearRatio, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Shuffleboard.update();
  }
}
