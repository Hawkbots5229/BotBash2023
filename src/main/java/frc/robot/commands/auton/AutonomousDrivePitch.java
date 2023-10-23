// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousDrivePitch extends CommandBase {

  private final DrivetrainSubsystem s_robotDrive;
  private final double pitch;
  private final double speed;

  /** Creates a new AutonomousPitch. */
  public AutonomousDrivePitch(DrivetrainSubsystem s_robotDrive, double pitch, double speed) {

    this.s_robotDrive = s_robotDrive;
    this.pitch = pitch;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_robotDrive.drive(speed, 0, 0, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotDrive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_robotDrive.getRobotPitch() > pitch;
  }
}
