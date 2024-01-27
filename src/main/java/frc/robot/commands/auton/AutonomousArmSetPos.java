// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AutonomousArmSetPos extends CommandBase {

  private final ArmSubsystem s_robotArm;
  private final ArmSubsystem.ArmPos pos;

   /** Creates a command to move the arm to a desired Location
   * 
   * @param s_robotArm Robot arm subsystem
   * @param pos Desired location of the arm (Enumeration)
   */
  public AutonomousArmSetPos(ArmSubsystem s_robotArm, ArmSubsystem.ArmPos pos) {
    
    this.s_robotArm = s_robotArm;
    this.pos = pos;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_robotArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.l_armPos.setTargetPosition(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_robotArm.setAngle(RobotContainer.l_armPos.getTargetPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_robotArm.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;}
}
