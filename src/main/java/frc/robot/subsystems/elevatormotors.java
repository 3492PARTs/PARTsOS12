package frc.robot.subsystems;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class elevatormotors extends SubsystemBase {

  private static final int deviceID = 1;
  private CANSparkMax extendmotor;
  private CANSparkMax pivotmotor;
 
  private double pivotMotorRatio = 0;
  private double shaftDiamitor = 3;
  private double otherGearRatio = 1;

  public elevatormotors(){
    // initialize SPARK MAX
    extendmotor = new CANSparkMax(deviceID, MotorType.kBrushless);
    pivotmotor = new CANSparkMax(deviceID, MotorType.kBrushless);
  }

public double getPivot(){
  return pivotmotor.getEncoder().getPosition()/pivotMotorRatio;
}

public double getPosition(){
  return (extendmotor.getEncoder().getPosition()*shaftDiamitor)/otherGearRatio;
}

public static elevatormotors getElevatormotorsInstance() {
  return null;
}

public void extendarm(double speed){
  extendmotor.set(speed);
}

public void pivotrotation(double rotation){
  pivotmotor.set(rotation);
}


}