// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public enum intakeDir {kIn, kOut, kOff};

  private final CANSparkMax m_left =
    new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_right =
    new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);

  private final SparkMaxPIDController pid_LeftVelControl = m_left.getPIDController();

  private final RelativeEncoder e_LeftEncoder = m_left.getEncoder();
  private final RelativeEncoder e_RightEncoder = m_right.getEncoder();
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    m_left.restoreFactoryDefaults();
    m_left.setInverted(IntakeConstants.kLeftMotorIntverted);
    m_left.setIdleMode(IntakeConstants.kIdleMode);
    m_left.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_left.setClosedLoopRampRate(IntakeConstants.kClosedLoopRampRate);
    m_left.setOpenLoopRampRate(IntakeConstants.kOpenLoopRampRate);
    m_left.enableVoltageCompensation(IntakeConstants.maxVoltage);
  
    
    m_right.restoreFactoryDefaults();
    m_right.setInverted(IntakeConstants.kRightMotorInverted);
    m_right.setIdleMode(IntakeConstants.kIdleMode);
    m_right.setSecondaryCurrentLimit(IntakeConstants.kCurrentLimit);
    m_right.setClosedLoopRampRate(IntakeConstants.kClosedLoopRampRate);
    m_right.setOpenLoopRampRate(IntakeConstants.kOpenLoopRampRate);
    m_right.enableVoltageCompensation(IntakeConstants.maxVoltage);

    pid_LeftVelControl.setFF(IntakeConstants.kFVel, IntakeConstants.kVelPidSlot);
    pid_LeftVelControl.setP(IntakeConstants.kPVel, IntakeConstants.kVelPidSlot);
    pid_LeftVelControl.setD(IntakeConstants.kDVel, IntakeConstants.kVelPidSlot);
    pid_LeftVelControl.setI(IntakeConstants.kIVel, IntakeConstants.kVelPidSlot);
  }

  public void setTargetOutput(double output) {
    m_left.set(output);
    m_right.set(output);
  }

  public void setTargetVelocity(double Velocity) {
    pid_LeftVelControl.setReference(
      Velocity,
      CANSparkMax.ControlType.kVelocity,
      IntakeConstants.kVelPidSlot);
  }

  public void wheelsIn() {
    setTargetOutput(IntakeConstants.kMaxOutput);
  }

  public void wheelsOut() {
    setTargetOutput(-IntakeConstants.kMaxOutput);
  }

  public double getIntakeVel() {
    return (e_LeftEncoder.getVelocity() + e_RightEncoder.getVelocity())/2;
  }

  public void stopMotors() {
    m_left.stopMotor();
    m_right.stopMotor();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

    e_LeftEncoder.setPosition(0);
    e_RightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Velocity", getIntakeVel());
    //System.out.println(m_left.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}