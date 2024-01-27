// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetPosCommand extends InstantCommand {

  private final ArmSubsystem.ArmPos pos;

  /** Creates a new Arm Set Position Command. 
   * This command drives the arm to a desired Location.
   * 
   * @param pos Location of the arm (Enumeration)
   */
  public ArmSetPosCommand(ArmSubsystem.ArmPos pos) {
    this.pos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.l_armPos.setTargetPosition(pos);
  }
}
