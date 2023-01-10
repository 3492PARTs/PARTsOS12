// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.Utils.Interfaces;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/** Add your docs here. */
public class TalonSRXDistanceValue implements EncoderValueInterface<TalonSRX>{
    TalonSRX talonSRX;
    double distPerRot;
    TalonSRXDistanceValue(TalonSRX talonSRX){
        this.talonSRX = talonSRX;
    }

    @Override
    public double getDistanceRaw() {

        return talonSRX.getSelectedSensorPosition();
    
    }
    
}
