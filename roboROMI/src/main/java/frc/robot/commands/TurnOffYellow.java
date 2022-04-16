// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.OnBoardIO;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnOffYellow extends CommandBase {
  private final OnBoardIO m_io;

  /**
   * Simple command to turn on the yellow LED on the Romi on board IO
   */
  public TurnOffYellow(OnBoardIO io) {
    m_io = io;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_io.setYellowLed(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_io.setYellowLed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
