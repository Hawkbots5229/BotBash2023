// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.tasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.AutonomousDriveStop;
import frc.robot.commands.auton.AutonomousIntakeSetSpd;
import frc.robot.commands.auton.AutonomousResetEncoders;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousDontMove extends SequentialCommandGroup {

  /** Creates a command sequence to:
   * 1. Reset drive and steering encoders
   * 2. Eject cube for claw intake
   * 3. Stops the robot
   * 
   * @param s_robotDrive Robot drivetrain subsystem
   * @param s_intake Robot intake subsystem
   */
  public AutonomousDontMove(DrivetrainSubsystem s_robotDrive, IntakeSubsystem s_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousResetEncoders(s_robotDrive),
      new AutonomousIntakeSetSpd(s_intake, -1.0, 0.2),
      new AutonomousDriveStop(s_robotDrive));
  }
}
