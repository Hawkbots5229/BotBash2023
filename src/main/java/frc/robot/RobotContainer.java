// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.dflt.ArmDefaultCommand;
import frc.robot.commands.dflt.DrivetrainDefaultCommand;
import frc.robot.commands.dflt.IntakeDefaultCommand;
import frc.robot.library.ArmController;
import frc.robot.commands.ArmSetPosCommand;
import frc.robot.commands.IntakeSetSpdCommand;
import frc.robot.commands.auton.AutonomousDriveStop;
import frc.robot.commands.auton.tasks.AutonomousChargeLine;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();
  private DrivetrainDefaultCommand drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_robotDrive);
  public static IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private IntakeDefaultCommand intakeDefaultCommand = new IntakeDefaultCommand(m_robotIntake);
  public static ArmSubsystem m_robotArm = new ArmSubsystem();
  private ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand(m_robotArm);
  public static ArmController l_armPos = new ArmController(ArmSubsystem.ArmPos.kHome);

  // The driver's controller
  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static XboxController m_mechController = new XboxController(OIConstants.kMechControllerPort);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> sc_autonSelect = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CameraServer.startAutomaticCapture("Drive Camera", 0);
    CameraServer.startAutomaticCapture("Claw Camera", 1);
    
    // Configure the button bindings
    configureButtonBindings();

    // Setup SmartDashboard Auton options
    sc_autonSelect.setDefaultOption("Don't Move", new AutonomousDriveStop(m_robotDrive));
    sc_autonSelect.addOption("Balance", new AutonomousChargeLine(m_robotDrive, m_robotIntake));
    SmartDashboard.putData("Auton Selection", sc_autonSelect);

    // Configure default commands
    m_robotDrive.setDefaultCommand(drivetrainDefaultCommand);
    m_robotIntake.setDefaultCommand(intakeDefaultCommand);
    m_robotArm.setDefaultCommand(armDefaultCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /** Intake Wheels: PovUp-Out PovDown-In */
    new POVButton(m_mechController, OIConstants.kUpDPad)
      .onTrue(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kIn))
      .onFalse(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOff));     
    new POVButton(m_mechController, OIConstants.kDownDPad)
      .onTrue(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOut))
      .onFalse(new IntakeSetSpdCommand(m_robotIntake, IntakeSubsystem.intakeDir.kOff));

    new JoystickButton(m_mechController, Button.kA.value)
      .onTrue(new ArmSetPosCommand(ArmSubsystem.ArmPos.kHome));
    new JoystickButton(m_mechController, Button.kB.value)
      .onTrue(new ArmSetPosCommand(ArmSubsystem.ArmPos.kExtend));
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return sc_autonSelect.getSelected();
  }
}
