// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

/** Add your docs here. */
public class SwerveData {

    public String name = "";
    public int steerCANId = 0;
    public int driveCANId = 0;
    public int encoderCANId = 0;
    public Boolean driveInvertValue = false;
    public Boolean steerInvertValue = false;
    public double steerAngleOffset = 0;
    public Boolean useAbsEnc = false;
    /***
     * 
     * @param name                 String name of this device
     * @param driveCANId           Drive CAN Id
     * @param driveInvertValue     Drive Invert of Motor
     * @param steerCANId           Steer CAN Id
     * @param steerInvertValue     Steer Invert of Motor
     * @param encoderCANId         Encoder CAN Id
     * @param encoderSernsorDir    Encoder Sensor Direction
     * @param steerAnalogOffset    Steer Analog Offset in Degrees.
     */
    public SwerveData(String name, int driveCANId,  Boolean driveInvertValue, int steerCANId, Boolean steerInvertValue, int encoderCANId, double steerAnalogOffset, Boolean useAbsEnc ){

        this.name = name;
        this.driveCANId = driveCANId;
        this.steerCANId = steerCANId;
        this.encoderCANId = encoderCANId;
        this.driveInvertValue = driveInvertValue;
        this.steerInvertValue = steerInvertValue;
        this.steerAngleOffset = steerAnalogOffset;
        this.useAbsEnc = useAbsEnc;
    }

}