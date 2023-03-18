// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PARTSlib2023.PARTS.frc.Utils.dataHolders.PIDValues;
import PARTSlib2023.PARTS.frc.commands.PIDTurn;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.PIDdrive;
import frc.robot.commands.Drivetrain.autoLevelNoPID;
import frc.robot.commands.Drivetrain.driveUntilAngle;
import frc.robot.commands.elevator.raiseArmAndDrop;
import frc.robot.subsystems.driveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class spinBalanceNoPID extends SequentialCommandGroup {
  /** Creates a new spinBalanceNoPID. */
  public spinBalanceNoPID() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(new raiseArmAndDrop(),new PIDTurn(driveTrain.geDriveTrain(), new PIDValues(0.001, 0.0005, 0), 167).withTimeout(2.5),new PIDdrive(driveTrain.geDriveTrain(), new PIDValues(3.75, .1, 0), Units.inchesToMeters(48)).withTimeout(2), new driveUntilAngle(), new PIDdrive(driveTrain.getDriveTrainInstance(), new PIDValues(2.55, .1, 0), Units.inchesToMeters(-87)),new PIDdrive(driveTrain.getDriveTrainInstance(), new PIDValues(2.55, .1, 0), Units.inchesToMeters(80)).withTimeout(3), new autoLevelNoPID()); 

  }
}
