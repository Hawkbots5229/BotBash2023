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

public class AutonomousBalance extends SequentialCommandGroup {
  
  /** Creates a command sequence to:
   * 1. Reset drive and steering encoders
   * 2. Eject cube for claw intake
   * 3. Drive forward until the robot begins climbing the charging station
   * 4. Drive forward until the robot is level
   * 5. Drive forward until the robot begins decending from the charging station
   * 6. Drive forward until the robot is level
   * 7. Drive forward until the robot crosses the line
   * 8. Drive backwards until the robot begins climbing the charging station
   * 9. Drive backwards until the robot is level
   * 10. Stop the robot
   * 
   * @param s_robotDrive Robot drivetrain subsystem
   * @param s_intake Robot intake subsystem
   */
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
