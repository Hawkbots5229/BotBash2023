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

  private final CANSparkMax m_rear =
    new CANSparkMax(ArmPivotConstants.kRearMotorPort, MotorType.kBrushless);

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

    m_rear.setInverted(ArmPivotConstants.kRearMotorInverted);
    m_rear.setIdleMode(ArmPivotConstants.kIdleMode);
    m_rear.enableVoltageCompensation(ArmPivotConstants.maxVoltage);
    m_rear.setSmartCurrentLimit(ArmPivotConstants.kCurrentLimit);

    resetEncoders();

    m_turningPIDController.setTolerance(ArmPivotConstants.kPosErrTolerance);
  }

  /** Sets the arm encoder to a position of 0. 
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.setPosition()
   * 
   */
  public void resetEncoders() {

    m_rearEncoder.setPosition(0);
  }

  /** Gets the current angle of the arm.
   * 
   * @return Current arm angle (Radians)
   * @param None
   * @implNote com.revrobotics.RelativeEncoder.getPosition()
   * @implNote ArmPivotConstants.kEncoderRevToArmRads
   */
  public double getAngle() {
    return m_rearEncoder.getPosition() * ArmPivotConstants.kEncoderRevToArmRads;
  }

  /** Sets target angle of the arm.
   *  Uses PID control to determine the motor speed requried to reach the target angle.
   * 
   * @return Void
   * @param tarAngle The desired angle of the arm (Radians)
   * @implNote edu.wpi.first.math.controller.ProfiledPIDController()
   * @implNote getAngle()
   * @implNote com.revrobotics.CANSparkMax.set()
   * 
   */
  public void setAngle(double tarAngle) {
    double output = m_turningPIDController.calculate(getAngle(), tarAngle);
    m_rear.set(output);
  }

  /** Stops the arm motor.
   * 
   * @return Void
   * @param None
   * @implNote com.revrobotics.CANSparkMax.stopMotor()
   * 
   */
  public void stopMotors() {

    m_rear.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmAngle", Math.toDegrees(getAngle()));
  }
}
