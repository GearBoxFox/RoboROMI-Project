// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/*
 * Creates a new TurnTime command. This command will turn your robot for a
 * desired rotational speed and time.
 */
public class PathweaverAuto extends CommandBase {
  Drivetrain m_drive;
  Trajectory pathFollow = new Trajectory();

  public PathweaverAuto(Drivetrain drivetrain) {
    m_drive = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  String trajJSON = "output/figureEight.wpilib.json";

  try{
    // Gets the path to the JSON file
     var trajPath = Filesystem.getDeployDirectory().toPath().resolve(trajJSON);
     // Converts JSON to trajectory
     pathFollow = TrajectoryUtil.fromPathweaverJson(trajPath);
   } catch(IOException ex){
     DriverStation.reportError("Unable to open trajectory: " + trajJSON, ex.getStackTrace());
   }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory robotPath = pathFollow;
    // Creates the Ramsete controller
    RamseteCommand ramseteCommand = new RamseteCommand(
      robotPath,
      m_drive::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drive::tankDriveVolts,
      m_drive
    );
    // Resets odometry in a crappy way
    m_drive.resetOdometry(robotPath.getInitialPose());
    
    // Returns the Ramsete controller command group, and then stops the drivetrain
    CommandScheduler.getInstance().schedule(ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0)));
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
