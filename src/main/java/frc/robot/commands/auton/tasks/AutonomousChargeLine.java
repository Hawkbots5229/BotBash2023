// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.tasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.AutonomousDriveDistance;
import frc.robot.commands.auton.AutonomousDrivePitch;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousChargeLine extends SequentialCommandGroup {
  /** Creates a new AutonomousHailMary. */
  public AutonomousChargeLine(DrivetrainSubsystem s_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousDriveDistance(s_robotDrive, 0, -0.9),
      new AutonomousDriveDistance(s_robotDrive, 0, 0.9),
      new AutonomousDriveDistance(s_robotDrive, 0, -0.9),
      new AutonomousDrivePitch(s_robotDrive, 86, -0.9),
      new AutonomousDriveDistance(s_robotDrive, 0, -0.9),
      new AutonomousDrivePitch(s_robotDrive, 86, -0.9),
      new AutonomousDriveDistance(s_robotDrive, 0, -0.9),
      new AutonomousDriveDistance(s_robotDrive, 0, 0.9),
      new AutonomousDrivePitch(s_robotDrive, 86, 0.5));    
  }
}
