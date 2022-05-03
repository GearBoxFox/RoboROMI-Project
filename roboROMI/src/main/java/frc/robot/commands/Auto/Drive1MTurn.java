package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Drive1MTurn extends SequentialCommandGroup {

    public Drive1MTurn(Drivetrain drive){
        super(Drive1M.drive(drive), Drive1M.turn(drive));
    }
    
}
