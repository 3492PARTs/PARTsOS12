// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.Utils.dataHolders;

/** Add your docs here. */
public class PIDValues {
    double[] PIDValues;
    /**
     * 
     * @param kp the tuned proportional gain (tune first )
     * @param ki tuned integral gain (tune kd first) last resort
     * @param kd tuned derivative gain (tune second)
     */
    public PIDValues(double kp, double ki, double kd){
        this.PIDValues[0] = kp;
        this.PIDValues[1] = ki;
        this.PIDValues[2] = kd;
    }

    public double[] getPIDValues() {
        return PIDValues;
    }

}
