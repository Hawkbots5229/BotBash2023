// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmController {

    private double targetPosition;
    private ArmSubsystem.ArmPos targetPositionEnum;

    /** Constructs an ArmController.
     *
     * @param pos Target Position of the arm (Radians)
     */
    public ArmController(ArmSubsystem.ArmPos pos) {
        this.targetPosition = updTargetPosition(pos); 
    }

    /** Updates target position of the arm. Throws AssertionError if the requested location is invalid.
     * 
     * @return Target Position of the arm (Radians)
     * @param pos Target Location of the arm (Enumuration)
     * @implNote ArmPivotConstants.kHomeLoc
     * @implNote ArmPivotConstants.kExtendLoc
     * @implNote ArmPivotConstants.kMidLoc
     */
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

    /** Sets the target position of the arm.
     * 
     * @return Void
     * @param pos Target Location of the arm (Enumeration)
     * @implNote updTargetPosition()
     */
    public void setTargetPosition(ArmSubsystem.ArmPos pos) {
        this.targetPosition = updTargetPosition(pos);
    }

    /** Gets the target position of the arm.
     * 
     * @return Target Position of the arm (Radians)
     * @param None
     */
    public double getTargetPosition() {
        return this.targetPosition;
    }

    /** Gets the target location of the arm.
     * 
     * @return Target Location of the arm (Enumeration)
     * @param None
     */
    public ArmSubsystem.ArmPos getTargetEnum() {
        return targetPositionEnum;
    }
}