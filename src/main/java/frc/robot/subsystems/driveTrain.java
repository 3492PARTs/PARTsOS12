// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import PARTSlib2023.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import PARTSlib2023.PARTS.frc.Utils.sensors.wheelLinearDistance;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class driveTrain extends beanieDriveTrain {

  private final Field2d m_field = new Field2d();


  // static CANSparkMax left1 = new CANSparkMax(18, MotorType.kBrushless);
  // static CANSparkMax left2 = new CANSparkMax(23, MotorType.kBrushless);

  // static CANSparkMax right1 = new CANSparkMax(24, MotorType.kBrushless);
  // static CANSparkMax right2 = new CANSparkMax(12, MotorType.kBrushless);

  static CANSparkMax left1 = new CANSparkMax(18, MotorType.kBrushless);
  static CANSparkMax left2 = new CANSparkMax(10, MotorType.kBrushless);
  static CANSparkMax left3 = new CANSparkMax(23, MotorType.kBrushless);
  

  static CANSparkMax right1 = new CANSparkMax(24, MotorType.kBrushless);
  static CANSparkMax right2 = new CANSparkMax(7, MotorType.kBrushless);
  static CANSparkMax right3 = new CANSparkMax(21, MotorType.kBrushless);

  DifferentialDriveKinematics dKinematics = new DifferentialDriveKinematics(1.421); // tbd
  DifferentialDrivePoseEstimator dEstimator = new DifferentialDrivePoseEstimator(dKinematics, getRotation(),
      leftDistance(), rightDistance(), new Pose2d(1.0, 3.0, getRotation()));
  private static driveTrain mDriveTrain = new driveTrain();
  private static double kv = 1.0051;
  private static double ka = .27947; // this is from old robot new one had low r squared but its 0.084872
  private static double ks = .20063;
  public static HashMap<String, Command> eventMap = new HashMap<>();

  /** Creates a new driveTrain. */
  public driveTrain() {
    super(new AHRS(), new MotorControllerGroup(left1,left2, left3), new MotorControllerGroup(right1,right2, right3));
    Shuffleboard.getTab("primary").add(m_field);

    left1.setOpenLoopRampRate(.85);
    left2.setOpenLoopRampRate(.85);
    left3.setOpenLoopRampRate(.85);

    right1.setOpenLoopRampRate(.85);
    right2.setOpenLoopRampRate(.85);
    right3.setOpenLoopRampRate(.85);

    left1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kBrake);
    left3.setIdleMode(IdleMode.kBrake);

    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);
    right3.setIdleMode(IdleMode.kBrake);

    left1.burnFlash();
    left2.burnFlash();
    left3.burnFlash();

    right1.burnFlash();
    right2.burnFlash();
    right3.burnFlash();

    Shuffleboard.getTab(Constants.debugTab).addNumber("leftDistance", leftDistanceDoubleSupplier());
    Shuffleboard.getTab(Constants.debugTab).addNumber("rightDistance", rightDistanceDoubleSupplier());

  }

  public beanieDriveTrain getInstance() {
    return mDriveTrain;
  }

  public static driveTrain getDriveTrainInstance() {
    return mDriveTrain;
  }

  public static driveTrain geDriveTrain() {
    return mDriveTrain;
  }

  @Override
  public Pose2d currentPose() {
    return dEstimator.getEstimatedPosition();
  }

  /**
   * @return in meters
   */
  @Override
  public double rightDistance() {
    // TODO Auto-generated method stub

    return -Units.inchesToMeters((right1.getEncoder().getPosition() * 6 * Math.PI) / 8.01);
  }

  private double wheelAverage(wheelLinearDistance[] wheelDistances) {

    double average = 0;
    for (wheelLinearDistance wheelDistance : wheelDistances) {
      average += wheelDistance.getDistanceMeters();
    }
    average = average / wheelDistances.length;
    return average;

  }

  /**
   * @return in meters
   */
  @Override
  public double leftDistance() {
    // TODO Auto-generated method stub
    return Units.inchesToMeters((left2.getEncoder().getPosition() * 6 * Math.PI) / 8.01);
  }

  public DoubleSupplier leftDistanceDoubleSupplier(){
    DoubleSupplier s = () -> leftDistance();
    return s;
  }

  public DoubleSupplier rightDistanceDoubleSupplier(){
    DoubleSupplier s = () -> rightDistance();
    return s;
  }

  public Supplier<Pose2d> getPoseSupplier() {
    Supplier<Pose2d> s = () -> dEstimator.getEstimatedPosition();
    return s;
  }

  /**
   * 
   * @return dKinematics is in meters
   */
  public DifferentialDriveKinematics getKinematics() {
    return dKinematics;
  }

  public void resetPose(Pose2d newPose) {

    dEstimator.resetPosition(getRotation(), leftDistance(), rightDistance(), newPose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dEstimator.update(getRotation(), leftDistance(), rightDistance());


    StringBuilder sBuilder = new StringBuilder();

    m_field.setRobotPose(dEstimator.getEstimatedPosition());
  }

  @Override
  public void autonomousSetup() {
    // TODO Auto-generated method stub

  }

  public BiConsumer<Double, Double> getBiConsumer() {
    BiConsumer<Double, Double> biC = (leftVoltage, rightVoltage) -> {
      moveVolts(leftVoltage, rightVoltage);//this used to invert the left side
    };
    return biC;
  }

  @Override
  public void teleopSetup() {
    // TODO Auto-generated method stub
    // driveTrain.getInstance().setDefaultCommand(new
    // joystickDrive(driveTrain.getInstance(), RobotContainer.driverController));
  }

  public void updatePoseVisually(Pair<Pose2d, Double> pose) {

    dEstimator.addVisionMeasurement(pose.getFirst(), pose.getSecond());

  }

  /**
   * 
   * @return in rotations per second
   */
  public double getLeftVelocity() {
    return (left2.getEncoder().getVelocity() / 60);
  }

  /**
   * 
   * @return in rotations per second
   */
  public double getRightVelocity() {
    return -right1.getEncoder().getVelocity() / 60;
  }

  /**
   * 
   * @return in meters per second
   */
  public Supplier<DifferentialDriveWheelSpeeds> getWheelSpeedSupplier() {
    Supplier<DifferentialDriveWheelSpeeds> s = () -> new DifferentialDriveWheelSpeeds(
        (Units.inchesToMeters(getLeftVelocity() * 6 * Math.PI) / 8.01),
        (Units.inchesToMeters(getRightVelocity() * 6 * Math.PI) / 8.01));
    return s;
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

    if (isFirstPath) {
      this.resetPose(traj.getInitialPose());
    }

    PPRamseteCommand controller1 = new PPRamseteCommand(
        traj,
        driveTrain.geDriveTrain().getPoseSupplier(),
        new RamseteController(),
        new SimpleMotorFeedforward(ks, kv, ka),
        driveTrain.geDriveTrain().getKinematics(),
        driveTrain.geDriveTrain().getWheelSpeedSupplier(),
        new PIDController(.06, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only
                                      // use feedforwards.
        new PIDController(.06, 0, 0),
        driveTrain.geDriveTrain().getBiConsumer(),
        true,
        this);

    return controller1; // TODO: should probably add the subsystem as a requirement to this command.
  }

}