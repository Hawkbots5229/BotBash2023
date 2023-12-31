// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;
import frc.robot.library.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(DriveConstants.SDFrontLeft);

  private final SwerveModule m_rearLeft =
      new SwerveModule(DriveConstants.SDRearLeft);

  private final SwerveModule m_frontRight =
      new SwerveModule(DriveConstants.SDFrontRight);

  private final SwerveModule m_rearRight =
      new SwerveModule(DriveConstants.SDRearRight);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public boolean isFieldRelative = false;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    resetEncoders();
    zeroHeading();
  }

  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean isFieldRelative, boolean optimize) {
    this.isFieldRelative = isFieldRelative;
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
          isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0], optimize, false, false);
    m_frontRight.setDesiredState(swerveModuleStates[1], optimize, false, false);
    m_rearLeft.setDesiredState(swerveModuleStates[2], optimize, false, false);
    m_rearRight.setDesiredState(swerveModuleStates[3], optimize, false, false);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void restDriveEncoders() {
    m_frontLeft.resetDriveEncoders();
    m_rearLeft.resetDriveEncoders();
    m_frontRight.resetDriveEncoders();
    m_rearRight.resetDriveEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * 
   * @return Distance in Inches
   */
  public double getDriveDistanceMeters(){
    double dis = (m_frontLeft.getDriveDistanceMeters() + m_rearLeft.getDriveDistanceMeters() + m_frontRight.getDriveDistanceMeters() + m_rearRight.getDriveDistanceMeters())/4.0;
    return dis;
  }
  public double getDriveDistanceInches(){
    return getDriveDistanceMeters() / DriveConstants.MetersPerInch;
  }

  public double getRobotAngle() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getRobotAngle360() {
    return (m_gyro.getAngle()%360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0) ;
  }

  public double getRobotPitch() {

    return m_gyro.getPitch(); 
  }

  public Rotation2d getRobotRotation2D(){
    double angle = getRobotAngle();
    return Rotation2d.fromDegrees(angle);
  }

  public void setFieldRelative(boolean frm){
    this.isFieldRelative = frm;
  }

  public void stopMotors(){
    m_frontLeft.stopMotors();
    m_rearLeft.stopMotors();
    m_frontRight.stopMotors();
    m_rearRight.stopMotors();

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

    //SmartDashboard.putNumber("RobotAngle", getRobotAngle360());
    //SmartDashboard.putBoolean("Relative ", isFieldRelative);
    //SmartDashboard.putNumber("RobotAngle360", getRobotAngle());
    SmartDashboard.putNumber("RobotPitch", getRobotPitch());
    m_frontLeft.sendData();
    m_frontRight.sendData();
    m_rearLeft.sendData();
    m_rearRight.sendData();
    
  }
}
