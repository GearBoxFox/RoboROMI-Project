// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import frc.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.TurnDegreesPID;
import frc.robot.commands.TurnOffYellow;
import frc.robot.commands.TurnOnYellow;
import frc.robot.commands.Auto.AutonomousDance;
import frc.robot.commands.Auto.AutonomousDistance;
import frc.robot.commands.Auto.AutonomousTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
    XboxButtonA.whenActive(new ResetGyro(m_drivetrain));

    // Setup trajectory chooser
    m_trajChooser.setDefaultOption("No Pathweaver", "Example");
    m_trajChooser.addOption("Figure Eight", "figureEight");
    SmartDashboard.putData(m_trajChooser);

    // Setup SmartDashboard options
    //m_chooser.setDefaultOption("Trajectory Control", getTraj());
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
      return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getLeftY(), () -> m_controller.getRightX());
        
  }
/*
  public Command getTrajectory(String trajName){
    System.out.println("In the function");
    String m_trajName = trajName;
    Trajectory trajectory = null;
    if(m_trajName == "Example"){
      System.out.println("in the If statement");
      // Voltage Constraint
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter), 
        DriveConstants.kDriveKinematics, 
      10);

    // Create a Trajectory Config
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxAccelerationMetersPerSecondSquared,AutoConstants.kMaxSpeedMetersPerSeconds)
    .setKinematics(DriveConstants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

    trajectory = TrajectoryGenerator.generateTrajectory( new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
        //new Translation2d(0.25, 0.25),
        //new Translation2d(0.5, -0.25),
        //new Translation2d(0.75, 0),
        //new Translation2d(0.5, 0.25),
        //new Translation2d(0.25, -0.25)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(0.5, 0, new Rotation2d(0)),
    // Pass config
    config);
    }
    // Creates an else statement for using a pathweaver JSON file
    else{
      // The location of the JSON file in a string format
      String trajJSON = "output/" + m_trajName + ".wpilib.json";
      System.out.println("in the Else statement");

      try{
        // Gets the path to the JSON file
        Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(trajJSON);
        // Converts JSON to trajectory
        trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
      } catch(IOException ex){
        DriverStation.reportError("Unable to open trajectory: " + trajJSON, ex.getStackTrace());
      }
    }
    
    // Creates the Ramsete controller
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );
    // Resets odometry in a crappy way
    m_drivetrain.resetOdometry(trajectory.getInitialPose());

    // Returns the Ramsete controller command group, and then stops the drivetrain
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }
  */

  /*
  public final Command getTraj(){
  Trajectory robotPath = Robot.path;
  // Creates the Ramsete controller
  RamseteCommand ramseteCommand = new RamseteCommand(
    robotPath,
    m_drivetrain::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                               DriveConstants.kvVoltSecondsPerMeter,
                               DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    m_drivetrain::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_drivetrain::tankDriveVolts,
    m_drivetrain
  );
  // Resets odometry in a crappy way
  m_drivetrain.resetOdometry(robotPath.getInitialPose());

  // Returns the Ramsete controller command group, and then stops the drivetrain
  return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }
  */
/*
  public Command printGyroAngle(){
    double angle = m_gryo.printRawAngle();
    return new PrintCommand(Double.toString(angle));
  }
  */
}
