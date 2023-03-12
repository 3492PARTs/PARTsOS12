
package frc.robot.commands.Drivetrain;

import PARTSlib2023.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import PARTSlib2023.PARTS.frc.Utils.dataHolders.PIDValues;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain;


public class PIDdrive extends CommandBase {
  /** Creates a new PIDDrive. */
  double init;
  double setPoint;
  driveTrain driveTrain;
  double[] pidValues;
  PIDController PIDController;

   /**
    * 
    * @param driveTrain a drivetrain implementing the proper interface
    * @param drivingValues tested pid values for a pid position based loop
    */
  public PIDdrive(driveTrain driveTrain, PIDValues drivingValues, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrain = driveTrain;
    this.pidValues = drivingValues.getPIDValues();
    this.setPoint = distance;
    PIDController = new PIDController(pidValues[0], pidValues[1], pidValues[2]);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    init = (driveTrain.getInstance().leftDistance() + driveTrain.getInstance().rightDistance()) /2;
    PIDController.setSetpoint(setPoint);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = PIDController.calculate(((driveTrain.getInstance().leftDistance() + driveTrain.getInstance().rightDistance()) /2) - init);

    speed = MathUtil.clamp(speed, -12, 12);

    driveTrain.moveVolts(speed, speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PIDDrive ended.");
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PIDController.atSetpoint() && driveTrain.getVelocityGyroXY() < 1; // todo: add velocity check
  }
}