// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elevatorState;

public class Elevator extends SubsystemBase {
  // angle, distance extended
  public static HashMap<elevatorState, Double> elevatorStateMap;
  

  double kS;
  double kG;
  double kV;
  double kA;

  PIDController velocityHolder = new PIDController(0, 0, 0);

  CANSparkMax pivotLeader;

  SparkMaxPIDController pivot1Controller;

  double pivotGearRatio = 32.0; // TODO: ask for gear ratio

  double wheelCircumference;

  elevatorState state;

  private static Elevator m_elevator = new Elevator();

  /** Creates a new Elevator. */
  // TODO: we need to add the ability for the arm to hold its position with a pid
  // loop, by using the second pid slot on the Spark Maxes we can have a velocity
  // control loop
  // alongside the control loop for position. this should be a high priority to
  // add.
  public Elevator() {

    state = elevatorState.home;
    System.out.println("made elevator");
    pivotLeader = new CANSparkMax(8, MotorType.kBrushless);

    pivotLeader.setSmartCurrentLimit(30);
    pivotLeader.setSecondaryCurrentLimit(30);
    pivot1Controller = pivotLeader.getPIDController();

    pivotLeader.setIdleMode(IdleMode.kBrake);
    pivotLeader.setInverted(true);

    velocityHolder.setSetpoint(0);


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

    elevatorStateMap.put(elevatorState.home, 0d);

    Shuffleboard.getTab(Constants.debugTab).addNumber("arm angle", getAngleSupplier());
    Shuffleboard.getTab(Constants.debugTab).addNumber("arm angular velocity", getAnglularVelocitySupplier());
    Shuffleboard.getTab(Constants.debugTab).add(velocityHolder);

  }

  public static Elevator getInstance() {
    return m_elevator;
  }

  public double getAngle() {
    return 360 * pivotLeader.getEncoder().getPosition() / pivotGearRatio;
  }

  public double getRotationRate() {
    return 360 * pivotLeader.getEncoder().getVelocity() / (pivotGearRatio * 60);

  }

  public DoubleSupplier getAngleSupplier() {
    DoubleSupplier s = () -> getAngle();
    return s;
  }

  public DoubleSupplier getAnglularVelocitySupplier() {
    DoubleSupplier s = () -> getRotationRate();
    return s;
  }

  public void incrementState(){
    if(state.getState() == 3){
      return;
    }
    this.state = elevatorState.values()[state.getState()+1];
  }

  public void decrementState(){
    if(state.getState() == 0){
      return;
    }
    this.state = elevatorState.values()[state.getState()-1];
  }

  public void setPivotSpeed(double speed) {
    pivotLeader.set(speed);
  }

  public void zeroPivotEncoder() {
    pivotLeader.getEncoder().setPosition(0);
  }

  public PIDController getPivotHolderController(){
    return velocityHolder;
  }

  /**
   * 
   * @param position the position commanded must be in rotations
   */
  public void setSetPointPivot(double position) {
    pivot1Controller.setReference(position * pivotGearRatio, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Shuffleboard.update();
  }
}
