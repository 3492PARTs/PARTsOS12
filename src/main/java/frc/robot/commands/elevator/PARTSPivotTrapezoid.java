package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Elevator;

public class PARTSPivotTrapezoid extends PARTSTrapezoidProfileCommand {
  /** Creates a new pivotTrapezoid. */
  public PARTSPivotTrapezoid(double goalAngle) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            Elevator.getInstance().getConstraints(),
            // Goal state
            new TrapezoidProfile.State(Math.toRadians(goalAngle), 0),
            // Initial state
            Elevator.getInstance().getState()
            ),
            Elevator.getInstance().getConstraints(),
            new TrapezoidProfile.State(Math.toRadians(goalAngle), 0),
            () -> Elevator.getInstance().getState(),

            
        state -> {
          // Use current trajectory state here
            Elevator.getInstance().setGoalState(state);
            double output = Elevator.getInstance().calcOutputVoltage(Elevator.getInstance().getGoalState().velocity);
            Elevator.getInstance().driveMotorVolts(output);
            System.out.println(Elevator.getInstance().getGoalState().velocity);
            System.out.println("pivoting like a PARTStrapezoid with a veloity commanded of" + state.velocity + " commanded output of  " + output);
        },
        Elevator.getInstance());
  }
}
