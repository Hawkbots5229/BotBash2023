// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousDriveStop extends InstantCommand {

  private final DrivetrainSubsystem s_robotDrive;

  /** Creates a command to stop the swerve drive and steering motors and reset their encoders. 
   * 
   * @param s_robotDrive Drivetrain subsystem
   */
  public AutonomousDriveStop(DrivetrainSubsystem s_robotDrive) {

    this.s_robotDrive = s_robotDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_robotDrive.resetEncoders();
    s_robotDrive.stopMotors();
  }
}