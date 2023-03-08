// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drivetrain.autoLevelNoPID;
import frc.robot.commands.Gripper.runGripper;
import frc.robot.commands.elevator.pivotController;
import frc.robot.commands.elevator.pivotTrapezoid;
import frc.robot.commands.elevator.zeroEncoder;
import frc.robot.commands.extender.linearController;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveTrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import PARTSlib2023.PARTS.frc.Utils.Controls.beanieController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
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


  // Replace with CommandPS4Controller or CommandJoystick if needed
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

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

    operatorController.y().whileTrue(new linearController(-.1));
    operatorController.a().whileTrue(new linearController(.1));

    operatorController.rightTrigger(.4).whileTrue(new runGripper(.5));
    operatorController.leftTrigger(.4).whileTrue(new runGripper(-1));

    operatorController.povUp().whileTrue(new pivotTrapezoid(30));

    


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveTrain.getDriveTrainInstance().followTrajectoryCommand(PathPlanner.loadPath("New New New New Path", new PathConstraints(1 ,.5)), true);
  }
}
