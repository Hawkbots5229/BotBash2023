// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.tasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.AutonomousDriveDistance;
import frc.robot.commands.auton.AutonomousDrivePitch;
import frc.robot.commands.auton.AutonomousDriveStop;
import frc.robot.commands.auton.AutonomousIntakeSetSpd;
import frc.robot.commands.auton.AutonomousResetEncoders;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousBalance extends SequentialCommandGroup {
  /** Creates a new AutonomousHailMary. */
  public AutonomousBalance(DrivetrainSubsystem s_robotDrive, IntakeSubsystem s_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousResetEncoders(s_robotDrive),
      new AutonomousIntakeSetSpd(s_intake, -1.0, 0.3),
      new AutonomousDriveDistance(s_robotDrive, 45, 1.0),
      new AutonomousDrivePitch(s_robotDrive, 87, 1.0),
      new AutonomousDriveDistance(s_robotDrive, 30, 1.0),
      new AutonomousDrivePitch(s_robotDrive, 87, 1.0),
      new AutonomousDriveDistance(s_robotDrive, 15, 1.0),
      new AutonomousDriveDistance(s_robotDrive, 60, -1.0),
      new AutonomousDrivePitch(s_robotDrive, 87, -0.3),
      new AutonomousDriveStop(s_robotDrive));    
  }
}
