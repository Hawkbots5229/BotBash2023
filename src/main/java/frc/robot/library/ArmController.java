// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import frc.robot.subsystems.ArmSubsystem;

/** Add your docs here. */
public class ArmController {

    private double targetPosition;

    public ArmController(ArmSubsystem.ArmPos pos) {
        this.targetPosition = updTargetPosition(pos); 
    }

    private double updTargetPosition(ArmSubsystem.ArmPos pos) {
        switch(pos) {
            case kHome: 
                return 0;
            case kExtend: 
                return 36; //40
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
}