// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmSubsystem;

/** Add your docs here. */
public class ArmController {

    private double targetPosition;
    private ArmSubsystem.ArmPos targetPositionEnum;

    public ArmController(ArmSubsystem.ArmPos pos) {
        this.targetPosition = updTargetPosition(pos); 
    }

    private double updTargetPosition(ArmSubsystem.ArmPos pos) {
        this.targetPositionEnum = pos;
        switch(pos) {
            case kHome: 
                return Math.toRadians(ArmPivotConstants.kHomeLoc);
            case kExtend: 
                return Math.toRadians(ArmPivotConstants.kExtendLoc);
            case kMid:
                return Math.toRadians(ArmPivotConstants.kMidLoc);
            default:
                throw new AssertionError("Illegal value: " + pos);   
        }
    }

    public void setTargetPosition(ArmSubsystem.ArmPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    public double getTargetPosition() {
        return this.targetPosition;
    }

    public ArmSubsystem.ArmPos getTargetEnum() {
        return targetPositionEnum;
    }
}