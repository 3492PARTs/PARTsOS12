// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.Utils.Interfaces;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class SparkMaxDistanceValue implements EncoderValueInterface<CANSparkMax> {
    double distPerRot;

    CANSparkMax sparkMax;
    SparkMaxDistanceValue(CANSparkMax sparkMax){
        this.sparkMax = sparkMax;

    }

    @Override
    public double getDistanceRaw() {
        // TODO Auto-generated method stub
        return sparkMax.getEncoder().getPosition();
    }



    @Override
    public void setConversionFactor(double distPerRot) {
        // TODO Auto-generated method stub
        this.distPerRot = distPerRot;
        
        
    }

    @Override
    public double getDistanceInches() {
        // TODO Auto-generated method stub
        return getDistanceRaw() * distPerRot;
    }



}
