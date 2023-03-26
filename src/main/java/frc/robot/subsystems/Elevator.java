// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  double kS;
  double kG;
  double kV;
  double kA;

  CANSparkMax pivotLeader;
  SparkMaxPIDController pivot1Controller;

  double pivotGearRatio = 32.0; // TODO: ask for gear ratio

  double pivotMountAngle = 73; // or 73 or 60

  double wheelCircumference;

  ArmFeedforward armholdFeedforward;
  TrapezoidProfile.Constraints ElevatorConstraints;
  TrapezoidProfile.State stateToBeExecuted = new TrapezoidProfile.State(0, 0);
  PIDController velocityPID = new PIDController(1.05, 8, .001); 
  private static Elevator m_elevator = new Elevator();

  /** Creates a new Elevator. */
  // TODO: we need to add the ability for the arm to hold its position with a pid
  // loop, by using the second pid slot on the Spark Maxes we can have a velocity
  // control loop
  // alongside the control loop for position. this should be a high priority to
  // add.
  public Elevator() {
    pivotLeader = new CANSparkMax(8, MotorType.kBrushless);
    ElevatorConstraints = new TrapezoidProfile.Constraints(Math.toRadians(30), Math.toRadians(6));// degrees and
                                                                                                   // degrees/s

    armholdFeedforward = new ArmFeedforward(0.039224, 0.31122, 0.012932);


    pivotLeader.setSmartCurrentLimit(30);
    pivotLeader.setSecondaryCurrentLimit(30);
    pivot1Controller = pivotLeader.getPIDController();

    pivotLeader.setIdleMode(IdleMode.kBrake);
    pivotLeader.setInverted(true);

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

    Shuffleboard.getTab("debug").addNumber("arm angle", getAngleSupplier());
    Shuffleboard.getTab("debug").addNumber("arm angular velocity", getAnglularVelocitySupplier());

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

  public TrapezoidProfile.Constraints getConstraints() {
    return ElevatorConstraints;
  }

  public TrapezoidProfile.State getState() {
    return new TrapezoidProfile.State(Math.toRadians(getAngle()), Math.toRadians(getAnglularVelocitySupplier().getAsDouble()));
  }

  public void setGoalState(TrapezoidProfile.State goalState) {
    this.stateToBeExecuted = goalState;
  }

  public TrapezoidProfile.State getGoalState() {
    return stateToBeExecuted;
  }

  public DoubleSupplier getAngleSupplier() {
    DoubleSupplier s = () -> getAngle();
    return s;
  }

  public double calcHoldingVoltage() {
    double output = (armholdFeedforward.calculate(Math.toRadians((pivotMountAngle + getAngle()) - 95.735), 0));
    return output;
  }

  public double calcOutputVoltage(double velocity) {
    double output = (armholdFeedforward.calculate(Math.toRadians((pivotMountAngle + getAngle()) - 95.735), velocity) + velocityPID.calculate(Math.toRadians(getRotationRate()), velocity));
    return output;
  }

  public void driveMotorVolts(double volts) {
    pivotLeader.setVoltage(volts);
  }

  public DoubleSupplier getAnglularVelocitySupplier() {
    DoubleSupplier s = () -> getRotationRate();
    return s;
  }

  public void setPivotSpeed(double speed) {
    pivotLeader.set(speed);
  }

  public void zeroPivotEncoder() {
    pivotLeader.getEncoder().setPosition(0);
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
