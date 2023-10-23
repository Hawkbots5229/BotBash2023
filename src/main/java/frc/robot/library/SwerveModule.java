// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.DriveConstants;;

public class SwerveModule {

  private final WPI_TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANCoder m_turningEncoderAbs;
  private final SwerveData m_data;
  private final RelativeEncoder m_turningEncoderRel;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveConstants.steerKp,
          SwerveConstants.steerKi,
          SwerveConstants.steerKd,
          new TrapezoidProfile.Constraints(
            SwerveConstants.steerMax_RadPS * DriveConstants.maxVoltage,
            SwerveConstants.steerMax_RadPSSq * DriveConstants.maxVoltage));

  /**
   * Constructs a SwerveModule.
   *
   * @param swerveData Data for swever module
   */
  public SwerveModule(SwerveData swerveData) {

    m_data = swerveData;
    //m_driveMotor = new WPI_TalonFX(data.driveCANId, "CANivore");
    m_driveMotor = new WPI_TalonFX(swerveData.driveCANId);
    m_turningMotor = new CANSparkMax(swerveData.steerCANId, MotorType.kBrushless);

    m_driveMotor.configFactoryDefault();

    if(swerveData.useAbsEnc) {
      m_turningEncoderAbs = new CANCoder(swerveData.encoderCANId, "CANivore");
      m_turningEncoderAbs.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
      m_turningEncoderAbs.configSensorDirection(swerveData.steerInvertValue);
    }
    else {
      m_turningEncoderAbs = null;
    }

    m_turningEncoderRel = m_turningMotor.getEncoder();
    m_turningMotor.setInverted(swerveData.steerInvertValue);

    m_driveMotor.setInverted(swerveData.driveInvertValue);
    
    m_driveMotor.configVoltageCompSaturation(DriveConstants.maxVoltage);

    m_driveMotor.enableVoltageCompensation(true);
    
    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_driveMotor.setSelectedSensorPosition(0);
    m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.kCurrentLimit, 80, 0.5));
    m_driveMotor.configOpenloopRamp(DriveConstants.kOpenLoopRampRate);

    m_turningMotor.enableVoltageCompensation(DriveConstants.maxVoltage);

    m_turningMotor.setIdleMode(IdleMode.kBrake);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(0.01);
  }


  /**
   * 
   * @return Angle in Radians of steer motor using CANCoder
   */
  public double getSwerveAngle(){
    return Math.toRadians(m_turningEncoderAbs.getAbsolutePosition());
  }

  public void resetDriveEncoders(){
    m_driveMotor.setSelectedSensorPosition(0);
  }

  public void resetSteerSensors(){
    if(m_data.useAbsEnc) {
      m_turningEncoderRel.setPosition((Math.toDegrees(getSwerveAngle())- m_data.steerAngleOffset)*SwerveConstants.kSteerMotCntsPerWheelDeg);
    }
    else {
      m_turningEncoderRel.setPosition(0);
    }
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    resetDriveEncoders();
    resetSteerSensors();
  }

  public double getDriveDistanceMeters(){
    final double dis = m_driveMotor.getSelectedSensorPosition();
    final double meters = dis / SwerveConstants.driveDistanceCntsPMeter;
    return meters;
  }

  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() / DriveConstants.MetersPerInch;
  }

  public double getDriveVelocity(){
    // Get the FalconFX velocity in raw units /100 ms
    double vel1 = m_driveMotor.getSelectedSensorVelocity();
    // Convert to Meters/s
    double velocity = vel1 / SwerveConstants.driveRawVelocityToMPS;
    return velocity;
  }

  public double getSteerMotorAngle(){
      return m_turningEncoderRel.getPosition() / SwerveConstants.steer_CntsPRad;   
  }

  public void stopMotors(){
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDriveDistanceMeters(), new Rotation2d(getSteerMotorAngle()));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(), new Rotation2d(getSteerMotorAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean optimize, boolean disableDrive, boolean disableSteer) {
    double steerOutput = 0;

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state;
    if(optimize){
        state = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerMotorAngle()));
    }else{
        state = desiredState;
    }

    if(!disableDrive){
      //System.out.println(state.speedMetersPerSecond / DriveConstants.maxSpeed);
      m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.maxSpeed);
    }
    // Calculate the turning motor output from the turning PID controller.
    if(!disableSteer){
        steerOutput = m_turningPIDController.calculate(getSteerMotorAngle(), state.angle.getRadians());
        m_turningMotor.set(steerOutput);
    }
  }

  public void sendData(){
    SmartDashboard.putNumber(m_data.name + "SteerMotorAngle", getSteerMotorAngle());
    //SmartDashboard.putNumber(m_data.name + "CANCoderAngle", Math.toDegrees(getSwerveAngle()));
    SmartDashboard.putNumber(m_data.name + "DriveDistance", getDriveDistanceInches());
  }
}
