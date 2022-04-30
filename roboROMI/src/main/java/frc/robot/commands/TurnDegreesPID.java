// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TurnDegreesPID extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_speed;
  private final PIDController pidTurn = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);
  

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesPID(double speed, double degrees, Drivetrain drive) {
    m_degrees = degrees;
    m_drive = drive;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
    System.out.println("Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   SmartDashboard.putNumber("Controller Output", pidTurn.calculate(m_drive.getGyroAngleZ(), m_degrees));
   /*
    if(pidcontrol.calculate(m_gyro.getAngleZ(), m_degrees) > m_degrees){
      m_drive.arcadeDrive(0, -m_speed);
    }
    if(pidcontrol.calculate(m_gyro.getAngleZ(), m_degrees) < m_degrees){
      m_drive.arcadeDrive(0, m_speed);
    }
  */
    double outputAngle = pidTurn.calculate(m_drive.getGyroAngleZ(), m_degrees);
    //double outputAngle = getCalcAngle(m_drive.getGyroAngleZ(), m_degrees);
    SmartDashboard.putNumber("Setpoint", outputAngle);
    //System.out.println(outputAngle);
    //System.out.println(m_drive.getGyroAngleX());
    //System.out.println("Before IF statement");
    if(outputAngle < 0){
      //System.out.println("Setpoint is less than zero");
      CommandScheduler.getInstance().schedule(new TurnDegrees(-m_speed, Math.abs(outputAngle), m_drive));
      }
    if(outputAngle >= 0){
      //System.out.println("Setpoint is greater than zero");
      CommandScheduler.getInstance().schedule(new TurnDegrees(m_speed, Math.abs(outputAngle), m_drive));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    // Compare distance travelled from start to distance based on degree turn
    //return getAverageTurningDistance() >= (inchPerDegree * m_degrees);
    return Math.abs(m_drive.getGyroAngleZ()) >= Math.abs(m_degrees);
  }

  /*
  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceInch());
    double rightDistance = Math.abs(m_drive.getRightDistanceInch());
    return (leftDistance + rightDistance) / 2.0;
  }

  private double getCalcAngle(double angle, double setpoint){
    double m_angle = angle;
    double m_setpoint = setpoint;
    double kp = 0.2;

    double error = m_angle - m_setpoint;

    return error * kp;
  }
  */
}
