package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.trajectory.TrajUtils;

public class Drive1M {    
    public static Command drive(Drivetrain drive){
        TrajectoryConfig config = TrajUtils.getDefaultTrajectoryConfig();
        return TrajUtils.startTrajectory("Paths\\forward", config, drive);
    }

    public static Command turn(Drivetrain drive){
        TrajectoryConfig config = TrajUtils.getDefaultTrajectoryConfig();
        return TrajUtils.createTrajectory("Paths\\turn", config, drive);
    }
}
