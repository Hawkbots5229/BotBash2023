// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousDriveDelay extends CommandBase {
  private final Timer tmr = new Timer();
  private final double delay;
  private final DrivetrainSubsystem s_robotDrive;

  /** Creates a new AutonomousDelay. */
  public AutonomousDriveDelay(DrivetrainSubsystem s_robotDrive, double delay) {

    this.s_robotDrive = s_robotDrive;
    this.delay = delay;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    tmr.reset();
    tmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotDrive.stopMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tmr.get() > delay;
  }
}
