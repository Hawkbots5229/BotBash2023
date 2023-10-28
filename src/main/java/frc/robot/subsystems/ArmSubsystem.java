// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmPos{kHome, kExtend, kMid};

  //private final CANSparkMax m_front = 
  //  new CANSparkMax(ArmPivotConstants.kFrontMotorPort, MotorType.kBrushless);

  private final CANSparkMax m_rear =
    new CANSparkMax(ArmPivotConstants.kRearMotorPort, MotorType.kBrushless);

  //private final RelativeEncoder m_frontEncoder = m_front.getEncoder();

  private final RelativeEncoder m_rearEncoder = m_rear.getEncoder();

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
  new ProfiledPIDController(
      ArmPivotConstants.kPPos,
      ArmPivotConstants.kIPos,
      ArmPivotConstants.kDPos,
      new TrapezoidProfile.Constraints(
        ArmPivotConstants.kMaxVel * ArmPivotConstants.maxVoltage,
        ArmPivotConstants.kMaxAcc * ArmPivotConstants.maxVoltage));

  /** Creates a new ArmPivotSubsystem. */
  public ArmSubsystem() {

    m_rear.restoreFactoryDefaults();

    //m_front.setInverted(ArmPivotConstants.kFrontMotorInverted);
    m_rear.setInverted(ArmPivotConstants.kRearMotorInverted);
    //m_front.setIdleMode(ArmPivotConstants.kIdleMode);
    m_rear.setIdleMode(ArmPivotConstants.kIdleMode);
    //m_front.enableVoltageCompensation(ArmPivotConstants.maxVoltage);
    m_rear.enableVoltageCompensation(ArmPivotConstants.maxVoltage);
    //m_front.setSmartCurrentLimit(ArmPivotConstants.kCurrentLimit);
    m_rear.setSmartCurrentLimit(ArmPivotConstants.kCurrentLimit);

    //m_front.follow(m_rear);

    resetEncoders();

    m_turningPIDController.setTolerance(ArmPivotConstants.kPosErrTolerance);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

    //m_frontEncoder.setPosition(0);
    m_rearEncoder.setPosition(0);
  }

  public void setAngle(double tarAngle) {
    double output = m_turningPIDController.calculate(getAngle(), tarAngle);
    m_rear.set(output);
  }

  public double getAngle() {
    return m_rearEncoder.getPosition() * ArmPivotConstants.kEncoderRevToArmRads;
  }

  /** Stops all drive motors */
  public void stopMotors() {

    //m_front.stopMotor();
    m_rear.stopMotor();
  }

  public void sendData(){
    SmartDashboard.putNumber("ArmAngle", Math.toDegrees(getAngle()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmAngle", Math.toDegrees(getAngle()));
  }
}
