// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousIntakeSetSpd extends CommandBase {

  private final Timer tmr = new Timer();
  private final IntakeSubsystem s_intake;
  private final double speed;
  private final double time;

  /** Creates a command to reset the swerve drive and steering encoders. 
   * 
   * @param s_intake Intake subsystem
   * @param speed Desired speed of the intake wheels. Range of -1 to 1.
   * @param time The length of time to run the intake. (Seconds)
   */
  public AutonomousIntakeSetSpd(IntakeSubsystem s_intake, double speed, double time) {

    this.s_intake = s_intake;
    this.speed = speed;
    this.time = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_intake);
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
    s_intake.setTargetOutput(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tmr.get() > time;
  }
}
