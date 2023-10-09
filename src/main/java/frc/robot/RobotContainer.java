// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.backUpandGrab;
import frc.robot.commands.backupAndGrabTurnLeft;
import frc.robot.commands.backupScoreLowBalance;
import frc.robot.commands.backupTravelGrab;
import frc.robot.commands.secondLevelScore;
import frc.robot.commands.throwInCommunityAndGrabAnother;
import frc.robot.commands.Drivetrain.PIDdrive;
import frc.robot.commands.Drivetrain.autoLevel;
import frc.robot.commands.Drivetrain.autoLevelNoPID;
import frc.robot.commands.Drivetrain.backupAndBalance;
import frc.robot.commands.Drivetrain.driveUntilAngle;
import frc.robot.commands.Gripper.runGripper;
import frc.robot.commands.elevator.pivotController;
import frc.robot.commands.elevator.pivotTrapezoid;
import frc.robot.commands.elevator.profiledPivot;
import frc.robot.commands.elevator.raiseArmAndDrop;
import frc.robot.commands.elevator.scoreHighAndHome;
import frc.robot.commands.elevator.zeroEncoder;
import frc.robot.commands.extender.PARTSTrapezoidProfileCommand;
import frc.robot.commands.extender.extend;
import frc.robot.commands.extender.linearController;
import frc.robot.commands.extender.linearTrapezoid;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.linearExtension;

import javax.security.auth.AuthPermission;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import PARTSlib2023.PARTS.frc.Utils.Controls.beanieController;
import PARTSlib2023.PARTS.frc.Utils.dataHolders.PIDValues;
import PARTSlib2023.PARTS.frc.commands.PIDDrive;
import PARTSlib2023.PARTS.frc.commands.PIDTurn;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static beanieController driveController = new beanieController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  //public static CommandGenericHID buttonBox = new CommandGenericHID(3);
  


  // Replace with CommandPS4Controller or CommandJoystick if needed
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("choose auto mode", autoChooser);
    autoChooser.addOption("Backup and b", new backUpandGrab());
    autoChooser.addOption("Backup and Grab Left", new backupAndGrabTurnLeft());
    autoChooser.addOption("backup and balance", new backupScoreLowBalance());
    autoChooser.addOption("backup, travel and balance", new backupTravelGrab());
    autoChooser.addOption("noPidBalance", new autoLevelNoPID());
    autoChooser.addOption("score level 2", new secondLevelScore());
    autoChooser.addOption("score level 3", new scoreHighAndHome());
    autoChooser.addOption("throw into community", new throwInCommunityAndGrabAnother());


    autoChooser.setDefaultOption("Backup and grab", new backUpandGrab());

    


    Shuffleboard.getTab("debug").add(new zeroEncoder());

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));


        
    // opera.getY().whileTrue(new pivotController(.4));
    // driveController.getA().whileTrue(new pivotController(-.05));

    // operatorController.y().whileTrue(new pivotController(.1));
    // operatorController.a().whileTrue(new pivotController(-.1));
    operatorController.x().whileTrue(new linearController(.2));
    operatorController.b().whileTrue(new linearController(-.2));

    operatorController.povDown().whileTrue(new profiledPivot(0).withTimeout(5));
    operatorController.povUp().whileTrue(new profiledPivot(40).withTimeout(3));

    operatorController.rightTrigger(.4).whileTrue(new runGripper(.5));
    operatorController.leftTrigger(.4).whileTrue(new runGripper(-1));
    operatorController.leftBumper().whileTrue(new runGripper(-.5));

    driveController.getX().and(driveController.getB()).whileTrue(new backupTravelGrab());
/* 
    buttonBox.button(6).whileTrue(new profiledPivot(0));
    buttonBox.button(5).whileTrue(new profiledPivot(40));
    buttonBox.button(3).whileTrue(new profiledPivot(50));
    buttonBox.button(4).whileTrue(new profiledPivot(70));    
*/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**k
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return driveTrain.getDriveTrainInstance().followTrajectoryCommand(PathPlanner.loadPath("New New New New Path", new PathConstraints(1 ,.5)), true);
    //return new pivotTrapezoid(40);
    //return new linearTrapezoid(6);
    //return new SequentialCommandGroup(new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(3.75, .1, 0), Units.inchesToMeters(-48)).withTimeout(5), new autoLevel());
    //return new SequentialCommandGroup(new pivotTrapezoid(50).withTimeout(5), new linearTrapezoid(12).withTimeout(12), new runGripper(-1).withTimeout(.5), new linearTrapezoid(0).withTimeout(10), new pivotTrapezoid(0));
    //return new SequentialCommandGroup(new extend(12d), new extend(0));  
    //return new linearTrapezoid(0);
    //return new SequentialCommandGroup(new linearTrapezoid(12), new linearTrapezoid(0));
    //return new SequentialCommandGroup(new raiseArmAndDrop(), new PIDdrive(driveTrain.getDriveTrainInstance(),new PIDValues(3.75, .1, 0), Units.inchesToMeters(-6)).withTimeout(2), new PIDTurn(driveTrain.getDriveTrainInstance(),new PIDValues(0.0014, 0.0005, 0) , 90d).withTimeout(3), new PIDdrive(driveTrain.getDriveTrainInstance(),new PIDValues(3.75, .1, 0), Units.inchesToMeters(90)).withTimeout(4), new PIDTurn(driveTrain.getDriveTrainInstance(),new PIDValues(0.0014, 0.0005, 0) , 90d).withTimeout(3) );
    //return new backupAndGrabTurnLeft();
    //return new driveUntilAngle();
    // return new backUpandGrab();
    return autoChooser.getSelected();
  }
}
