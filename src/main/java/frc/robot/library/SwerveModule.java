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
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.DriveConstants;;

public class SwerveModule {

  private final WPI_TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANCoder m_turningEncoderAbs;
  private final SwerveData m_swerveData;
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

  /** Constructs a SwerveModule.
   *
   * @param swerveData Data for swever module
   */
  public SwerveModule(SwerveData swerveData) {

    m_swerveData = swerveData;
    m_driveMotor = new WPI_TalonFX(swerveData.driveCANId);
    m_turningMotor = new CANSparkMax(swerveData.steerCANId, MotorType.kBrushless);

    if(swerveData.useAbsEnc) {
      m_turningEncoderAbs = new CANCoder(swerveData.encoderCANId);
      m_turningEncoderAbs.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
      m_turningEncoderAbs.configSensorDirection(swerveData.steerEncoderInvert);
    }
    else {
      m_turningEncoderAbs = null;
    }

    m_turningEncoderRel = m_turningMotor.getEncoder();

    m_driveMotor.configFactoryDefault();
    m_driveMotor.setInverted(swerveData.driveMotorInvert);
    m_driveMotor.configVoltageCompSaturation(DriveConstants.maxVoltage);
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.setNeutralMode(DriveConstants.driveMode);
    m_driveMotor.setSelectedSensorPosition(0);
    m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.kCurrentLimit, 80, 0.5));
    m_driveMotor.configOpenloopRamp(DriveConstants.kOpenLoopRampRate);

    m_turningMotor.enableVoltageCompensation(DriveConstants.maxVoltage);
    m_turningMotor.setInverted(swerveData.steerMotorInvert);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous. 
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(0.01);
  }

  /** Gets the absolute position of the swerve wheels
   * 
   * @return Angle (Radians)
   * @param None
   * @implNote com.ctre.phoenix.sensors.CANCoder.getAbsolutePosition()
   */
  public double getSwerveAngle(){
    return Math.toRadians(m_turningEncoderAbs.getAbsolutePosition());
  }

  /** Gets the relative position of the swerve wheels.
   * 
   * @return Angle (Revolutions)
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.getPosition()
   */
  public double getSteerAngle(){
    return m_turningEncoderRel.getPosition();
  }

  /** Resets the drive motor encoders to a position of 0.
   * 
   * @return Void
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.BaseMotorController.setSelectedSensorPosition()
   */
  public void resetDriveEncoders(){
    m_driveMotor.setSelectedSensorPosition(0);
  }

  /** Sets the relative steering motor encoders to the absolute position or
   *  0 if there is no absolute position.
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.setPosition()
   * @implNote getSwerveAngle()
   * @implNote SwerveConstants.steer_CntsPRad
   */
  public void resetSteerSensors(){
    if(m_swerveData.useAbsEnc) {
      m_turningEncoderRel.setPosition((getSwerveAngle()- Math.toRadians(m_swerveData.steerAngleOffset))*SwerveConstants.steer_CntsPRad);
    }
    else {
      m_turningEncoderRel.setPosition(0);
    }
  }

  /** Resets both the relative drive and steering motor encoders
   * 
   * @return Void
   * @param None
   * @implNote resetDriveEncoders()
   * @implNote resetSteerSensors()
   */
  public void resetEncoders() {
    resetDriveEncoders();
    resetSteerSensors();
  }

  /** Gets the distance driven in meters
   * 
   * @return Distance (meters)
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.BaseMotorController.getSelectedSensorPosition()
   * @implNote SwerveConstants.driveDistanceCntsPMeter
   */
  public double getDriveDistanceMeters(){
    final double dis = m_driveMotor.getSelectedSensorPosition();
    final double meters = dis / SwerveConstants.driveDistanceCntsPMeter;
    return Math.abs(meters);
  }

  /** Gets the distance driven in inches
   * 
   * @return Distance (inches)
   * @param None
   * @implNote getDriveDistanceMeters()
   * @implNote DriveConstants.MetersPerInch
   */
  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() / DriveConstants.MetersPerInch;
  }

  /** Gets the velocity of the drive wheel
   * 
   * @return Velocity (meters/sec)
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.BaseMotorController.getSelectedSensorVelocity()
   * @implNote SwerveConstants.driveRawVelocityToMPS
   */
  public double getDriveVelocity(){
    double vel1 = m_driveMotor.getSelectedSensorVelocity();
    double velocity = vel1 / SwerveConstants.driveRawVelocityToMPS;
    return velocity;
  }

  /** Gets the angle of the drive wheel
   * 
   * @return Angle (Radians)
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.getPosition()
   * @implNote SwerveConstants.steer_CntsPRad
   */
  public double getSteerMotorAngle(){
      return m_turningEncoderRel.getPosition() / SwerveConstants.steer_CntsPRad;   
  }

   /** Stop drive and steering motors
   * 
   * @return Void
   * @param None
   * @implNote com.ctre.phoenix.motorcontrol.can.WPI_TalonFX.stopMotor()
   * @implNote com.revrobotics.CANSparkMax.stopMotor()
   */
  public void stopMotors(){
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }

  /** Returns the current position of the swerve module as a swerve module position structure.
   *
   * @return Swerve Module Position
   * @param None
   * @implNote edu.wpi.first.math.kinematics.SwerveModulePosition()
   * @implNote edu.wpi.first.math.geometry.Rotation2d()
   * @implNote getDriveDistanceMeters()
   * @implNote getSteerMotorAngle()
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDriveDistanceMeters(), new Rotation2d(getSteerMotorAngle()));
  }

  /** Returns the current state of the swerve module as a swerve module state structure.
   * 
   * @return Swerve Module State
   * @param None
   * @implNote edu.wpi.first.math.kinematics.SwerveModuleState()
   * @implNote edu.wpi.first.math.geometry.Rotation2d()
   * @implNote getDriveVelocity()
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(), new Rotation2d(getSteerMotorAngle()));
  }

  /** Sets the desired state for the module.
   * 
   * @return Void
   * @param desiredState Desired state with speed and angle.
   * @param optimize Whether to optimize steering
   * @param disableDrive Whether to disable drive for testing
   * @param disableSteer Whether to disable steering for testing
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
      m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.maxSpeed);
    }
    // Calculate the turning motor output from the turning PID controller.
    if(!disableSteer){
        steerOutput = m_turningPIDController.calculate(getSteerMotorAngle(), state.angle.getRadians());
        m_turningMotor.set(steerOutput);
    }
  }

  /** Puts swerve data to the dashboard
   * 
   * @return Void
   * @param None
   */
  public void sendData(){
    SmartDashboard.putNumber(m_swerveData.name + "SteerMotorAngle", getSteerAngle());
    SmartDashboard.putNumber(m_swerveData.name + "CANCoderAngle", Math.toDegrees(getSwerveAngle()));
    SmartDashboard.putNumber(m_swerveData.name + "DriveDistance", getDriveDistanceInches());
    SmartDashboard.putNumber(m_swerveData.name + "DriveVelocity", getDriveVelocity());
  }
}
