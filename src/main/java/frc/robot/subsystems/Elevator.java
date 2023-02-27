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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  double kS;
  double kG;
  double kV;
  double kA;

  CANSparkMax pivotLeader;
  CANSparkMax linearMotor;
  SparkMaxPIDController pivot1Controller;
  SparkMaxPIDController linear1Controller;

  TalonSRX leftGripper;
  TalonSRX rightGripper;

  double pivotGearRatio = 32.0; //TODO: ask for gear ratio
  double linearGearRatio = 4.0;

  double wheelCircumference;

  private static Elevator m_elevator = new Elevator();

  /** Creates a new Elevator. */
  //TODO: we need to add the ability for the arm to hold its position with a pid loop, by using the second pid slot on the Spark Maxes we can have a velocity control loop
  // alongside the control loop for position. this should be a high priority to add.
  public Elevator() {
    pivotLeader = new CANSparkMax(8, MotorType.kBrushless);
    linearMotor = new CANSparkMax(9, MotorType.kBrushless);

    pivotLeader.setSmartCurrentLimit(30);
    pivotLeader.setSecondaryCurrentLimit(30);
    pivot1Controller = pivotLeader.getPIDController();
    linear1Controller = linearMotor.getPIDController();

    linearMotor.setSmartCurrentLimit(15, 15);

    linear1Controller = linearMotor.getPIDController();

    pivotLeader.setIdleMode(IdleMode.kBrake);
    pivotLeader.setInverted(true);
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

    Shuffleboard.getTab("debug").addNumber("arm angle", getAngleSupplier());
    Shuffleboard.getTab("debug").addNumber("arm angular velocity", getAnglularVelocitySupplier());
    Shuffleboard.getTab("debug").addNumber("arm Linear Extension", getExtensionDistanceSupplier());
    Shuffleboard.getTab("debug").addNumber("arm linear velocity", getExtensionRateSupplier());

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

  public void setPivotSpeed(double speed) {
    pivotLeader.set(speed);
  }

  public void setLinearSpeed(double speed) {
    linearMotor.set(speed);
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

  public void setSetPointLinear(double extensionMeters) {
    linear1Controller.setReference(extensionMeters * linearGearRatio, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Shuffleboard.update();
  }
}
