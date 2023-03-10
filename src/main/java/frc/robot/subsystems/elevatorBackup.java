// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class elevatorBackup extends TrapezoidProfileSubsystem {
  /** Creates a new elevatorBackup. */
  public elevatorBackup() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(Math.toRadians(30), Math.toRadians(10)),
        // The initial position of the mechanism
        0);
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
  }
}
