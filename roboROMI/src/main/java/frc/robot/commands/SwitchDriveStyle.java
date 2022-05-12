package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwitchDriveStyle extends CommandBase{
    Drivetrain m_drive;

    public SwitchDriveStyle(Drivetrain drive){
        m_drive = drive;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_drive.arcadeStyle = !m_drive.arcadeStyle;
    }

    @Override
    public void end(boolean finished){}

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
