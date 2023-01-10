// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.Utils.sensors;

import com.PARTS.frc.Utils.Interfaces.EncoderValueInterface;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public abstract class wheelLinearDistance {
    private double gearRatio;
    private double wheelCircumference;
    EncoderValueInterface<?> encoder;

    /**
     * 
     * @param gearRatio the gear ratio as a decimal value
     * @param wheelCircumference the wheel circumference in inches
     */
    public wheelLinearDistance(Double gearRatio, Double wheelCircumference, EncoderValueInterface<?> eInterface){
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;
        this.encoder = eInterface;
        
    }

    public double getDistanceMeters(){
        return Units.inchesToMeters((encoder.getDistanceRaw() * wheelCircumference) / gearRatio);
    }
}
