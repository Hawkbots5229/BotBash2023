// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dflt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends CommandBase {

  IntakeSubsystem s_robotIntake;

  /** Creates a new Intake Default Command. 
   * This command will keep the intake in its default state of off.
   * 
   * @param s_robotIntake Intake subsystem
   */
  public IntakeDefaultCommand(IntakeSubsystem s_robotIntake) {
    addRequirements(s_robotIntake);
    this.s_robotIntake = s_robotIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotIntake.stopMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
