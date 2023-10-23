// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.library.SwerveData;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 42;
    public static final int kRearLeftDriveMotorPort = 40;
    public static final int kFrontRightDriveMotorPort = 43;
    public static final int kRearRightDriveMotorPort = 41;

    public static final int kFrontLeftTurningMotorPort = 32;
    public static final int kRearLeftTurningMotorPort = 30;
    public static final int kFrontRightTurningMotorPort = 33;
    public static final int kRearRightTurningMotorPort = 31;

    public static final int kFrontLeftTurningEncoderPorts = 52;
    public static final int kRearLeftTurningEncoderPorts = 50;
    public static final int kFrontRightTurningEncoderPorts = 53;
    public static final int kRearRightTurningEncoderPorts = 51;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.43815;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.65405;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double maxVoltage = 12.0;
    public static final double maxSpeed = 4.36; // m/s
    public static final double maxAngularSpeed = Math.PI*4; // 1/2 Rotation/Sec
    public static final double rotKp = 0.01;
    public static final double rotKi = 0.02;
    public static final double rotKd = 0.0;
    public static final double rotToleranceDeg = 1;
    public static final double rotToleranceVel = 10; // Deg/sec
    public static final double rotMaxOutput  = .5;
    public static final double stickDeadband = 0.1;
    public static final double speedScale = 1.0;
    public static final double rotationScale = 0.5;
    public static final double autoRotateOutScale = 1.0;
    public static final double MetersPerInch = 1/39.37008;

    public static final boolean kGyroReversed = true;

    public static final SwerveData SDFrontLeft = new SwerveData("FL", 
    kFrontLeftDriveMotorPort, 
    kFrontLeftDriveEncoderReversed, 
    kFrontLeftTurningMotorPort, 
    kFrontLeftTurningEncoderReversed, 
    kFrontLeftTurningEncoderPorts,
    0,
    false);

  public static final SwerveData SDRearLeft = new SwerveData("RL", 
    kRearLeftDriveMotorPort, 
    kRearLeftDriveEncoderReversed, 
    kRearLeftTurningMotorPort, 
    kRearLeftTurningEncoderReversed, 
    kRearLeftTurningEncoderPorts,
    0,
    false);

  public static final SwerveData SDFrontRight = new SwerveData("FR", 
    kFrontRightDriveMotorPort, 
    kFrontRightDriveEncoderReversed, 
    kFrontRightTurningMotorPort, 
    kFrontRightTurningEncoderReversed, 
    kFrontRightTurningEncoderPorts,
    0,
    false);

  public static final SwerveData SDRearRight = new SwerveData("RR", 
    kRearRightDriveMotorPort, 
    kRearRightDriveEncoderReversed, 
    kRearRightTurningMotorPort, 
    kRearRightTurningEncoderReversed, 
    kRearRightTurningEncoderPorts, 
    0,
    false);
  }

  public static final class SwerveConstants {

    public static final double steerKp = 0.55;
    public static final double steerKi = 0.15;
    public static final double steerKd = 0;

    public static final double steerSMFKs = 0.0;
    public static final double steerSMFKv = 0;
    public static final double steerSMFKa = 0;

    public static final double steerMax_RadPS = Math.PI;
    public static final double steerMax_RadPSSq = Math.pow(steerMax_RadPS,2);
    public static final double steer_CntsPRad = 13.7/(2.0*Math.PI);

    public static final double driveKp = 0.5;
    public static final double driveKi = 1.5;
    public static final double driveKd = 0;

    public static final double driveSMFKs = 0.0;
    public static final double driveSMFKv = DriveConstants.maxVoltage/DriveConstants.maxSpeed;
    public static final double driveSMFKa = 0;

    public static final double driveDistanceCntsPMeter = 49907;
    public static final double driveRawVelocityToMPS = 4990.68;

    public static final double kSteerMotEncoderCountsPerRev = 2048.0;
    public static final double kSteerRatio = 15.43;
    public static final double kSteerMotCntsPerWheelDeg = (kSteerMotEncoderCountsPerRev * kSteerRatio) / 360;
    public static final double kSteerMotCountsPerWheelRadian = (kSteerMotEncoderCountsPerRev / (2 * Math.PI)) * kSteerRatio;
    public static final double kSteerEncoderCountsPerRev = 4096.0;
    public static final double kSteerCountsPerRadian = kSteerEncoderCountsPerRev / 2 * Math.PI;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechControllerPort = 1;

    public static final int kUpDPad = 180;
    public static final int kDownDPad = 0;
  }

  public static class IntakeConstants{

    public static final int kLeftMotorPort = 20;
    public static final int kRightMotorPort = 21;

    public static final boolean kLeftMotorIntverted = true;
    public static final boolean kRightMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 40;
    public static final double kClosedLoopRampRate = 1.5;
    public static final double kOpenLoopRampRate = 40;
    public static final double kMaxOutput = 0.9;
        
    public static final double kGearBoxRatio = 1/7.0;
    public static final double kEncoderRpmToWheelRpm = kGearBoxRatio;

    public static final int kVelPidSlot = 0;
    public static final double kFVel = 1;
    public static final double kPVel = 0;
    public static final double kIVel = 0;
    public static final double kDVel = 0;

    public static final double kMaxVel = 1; //Wheel RPM
  }

  public static class ArmPivotConstants {

    public static final int kFrontMotorPort = 22;
    public static final int kRearMotorPort = 23;

    public static final boolean kFrontMotorInverted = false;
    public static final boolean kRearMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double maxVoltage = 12.0;
    public static final int kCurrentLimit = 40;
   
    public static final double kGearBoxRatio = (1/12.75);
    public static final double kSprocketRatio = (22.0/38.0); 
    public static final double kEncoderRevToArmRads = kGearBoxRatio*kSprocketRatio*2*Math.PI;

    public static final double kPPos = 0.41; //0.5
    public static final double kIPos= 0.01;
    public static final double kDPos = 0;
    public static final double kPosErrTolerance = 10;

    public static final double kMaxVel = Math.PI; //RadPerSec
    public static final double kMaxAcc = Math.pow(kMaxVel,2);; //RadPerSecSqrd
  }

  public static final class AutoConstants {

  }
}
