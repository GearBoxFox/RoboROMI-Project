// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DoubleDriveStyle;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SwitchDriveStyle;
import frc.robot.commands.TurnDegreesPID;
import frc.robot.commands.TurnOffYellow;
import frc.robot.commands.TurnOnYellow;
import frc.robot.commands.Auto.AutonomousDance;
import frc.robot.commands.Auto.AutonomousDistance;
import frc.robot.commands.Auto.AutonomousTime;
import frc.robot.commands.Auto.Drive1M;
import frc.robot.commands.Auto.Drive1MTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  public final XboxController m_controller = new XboxController(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Creates chooser for trajectory routines
  private final SendableChooser<String> m_trajChooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    JoystickButton XboxBumperLeft = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    JoystickButton XboxBumperRight = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    XboxBumperLeft.or(XboxBumperRight).whenActive(new TurnOnYellow(m_onboardIO));
    XboxBumperLeft.or(XboxBumperRight).whenInactive(new TurnOffYellow(m_onboardIO));

    JoystickButton XboxButtonA = new JoystickButton(m_controller, XboxController.Button.kA.value);
    XboxButtonA.whenActive(new SwitchDriveStyle(m_drivetrain));

    // Setup trajectory chooser
    m_trajChooser.setDefaultOption("No Pathweaver", "Example");
    m_trajChooser.addOption("Figure Eight", "figureEight");
    SmartDashboard.putData(m_trajChooser);

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Trajectory Control", new Drive1MTurn(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    m_chooser.addOption("Auto Dance", new AutonomousDance(m_drivetrain));
    m_chooser.addOption("PID Turn", new TurnDegreesPID(1, 90, m_drivetrain));
    m_chooser.addOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
      return new DoubleDriveStyle(
        m_drivetrain, () -> -m_controller.getLeftY(), () -> m_controller.getRightX(), () -> m_controller.getRightY());
        
  }

}
