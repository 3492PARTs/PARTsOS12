// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import PARTSlib2023.PARTS.frc.Utils.dataHolders.PIDValues;
import PARTSlib2023.PARTS.frc.commands.PIDDrive;
import PARTSlib2023.PARTS.frc.commands.PIDTurn;
import PARTSlib2023.PARTS.frc.commands.joystickDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.angleTurn;
import frc.robot.commands.Drivetrain.autoLevel;
import frc.robot.commands.Drivetrain.autoLevelNoPID;
import frc.robot.commands.Gripper.holdGripper;
import frc.robot.commands.elevator.holdElevator;
import frc.robot.commands.elevator.runElevatorTeleop;
import frc.robot.commands.extender.stopExtender;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.cameraSystem;
import frc.robot.subsystems.driveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    NetworkTableInstance.getDefault().getEntry("/CameraPublisher/FishEyes/streams").setStringArray(new String[]{"mjpg:http://10.34.92.2:8008/?action=stream"});
    driveTrain.geDriveTrain().calibrateGyro();
    Gripper.getInstance().setDefaultCommand(new holdGripper());
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // new autoLevelNoPID().schedule();

    //new PIDTurn(driveTrain.getDriveTrainInstance(), new PIDValues(0.0014, 0.0005, 0), 90).schedule();;
    //new PIDDrive(driveTrain.getDriveTrainInstance(), new PIDValues(49.94, 0, 2.775));
    //Elevator.getInstance().setDefaultCommand(new holdElevator());
    //TODO:test this change

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Elevator.getInstance().setDefaultCommand(new runElevatorTeleop(RobotContainer.operatorController));

    driveTrain.geDriveTrain().setDefaultCommand(new joystickDrive(driveTrain.getDriveTrainInstance(), RobotContainer.driveController));
    new stopExtender().schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //cameraSystem.getCameraSystem().changeCamera(RobotContainer.driveController);
    //TODO:test this change

    

    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    driveTrain.geDriveTrain().setDefaultCommand(new joystickDrive(driveTrain.geDriveTrain(), RobotContainer.driveController));
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
