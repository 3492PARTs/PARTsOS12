// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.PARTS.frc.Utils.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class BeanieLimeLight {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    /**
     * @apiNote currently no arguments because we've always used the default settings 
     */
    public BeanieLimeLight(){



    }

    public double getTa() {
        return ta.getDouble(0);
    }

    public double getTx() {
        return tx.getDouble(0);
    }

    public double getTy() {
        return ty.getDouble(0);
    }


}
