// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that runs a {@link TrapezoidProfile}. Useful for smoothly controlling mechanism motion.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class PARTSTrapezoidProfileCommand extends CommandBase {
  private  TrapezoidProfile m_profile;
  private final Consumer<State> m_output;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal;
  private Supplier<TrapezoidProfile.State> stateSupplier;


  private final Timer m_timer = new Timer();

  /**
   * Creates a new TrapezoidProfileCommand that will execute the given {@link TrapezoidProfile}.
   * Output will be piped to the provided consumer function.
   *
   * @param profile The motion profile to execute.
   * @param output The consumer for the profile output.
   * @param requirements The subsystems required by this command.
   */
  public PARTSTrapezoidProfileCommand(
      TrapezoidProfile profile, TrapezoidProfile.Constraints constraints, TrapezoidProfile.State goal, Supplier<TrapezoidProfile.State> stateSupplier, Consumer<State> output, Subsystem... requirements) {
    m_profile = requireNonNullParam(profile, "profile", "TrapezoidProfileCommand");
    m_output = requireNonNullParam(output, "output", "TrapezoidProfileCommand");
    this.constraints = constraints;
    this.goal = goal;
    this.stateSupplier = stateSupplier;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_profile = new TrapezoidProfile(constraints, goal, stateSupplier.get());
    m_timer.restart();
  
  }

  @Override
  public void execute() {
    m_output.accept(m_profile.calculate(m_timer.get()));
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_profile.totalTime());
  }
}