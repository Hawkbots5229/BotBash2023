// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.tasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.AutonomousDriveDistance;
import frc.robot.commands.auton.AutonomousIntakeSetSpd;
import frc.robot.commands.auton.AutonomousResetEncoders;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCrossLine extends SequentialCommandGroup {
  /** Creates a new AutonomousCrossLine. */
  public AutonomousCrossLine(DrivetrainSubsystem s_robotDrive, IntakeSubsystem s_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousResetEncoders(s_robotDrive),
      new AutonomousIntakeSetSpd(s_intake, -1.0, 0.2),
      new AutonomousDriveDistance(s_robotDrive, 150, 0.8)
    );
  }
}
