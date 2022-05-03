package frc.robot.trajectory;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.StringTokenizer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Auto.FollowTraj;
import frc.robot.commands.Auto.SetInitialOdometryCommand;
import frc.robot.subsystems.Drivetrain;

public class TrajUtils {

    //Constants for defualt trajectory config
    public static double defualtVelocity = 1;
    public static double defaultAccel = 1;

    //Constants for slower trajectory config
    public static double slowerVelocity = 0.5;
    public static double slowerAccel = 0.5;

    public static Trajectory loadTrajectory(String filename, TrajectoryConfig trajectoryConfig){
        TrajectoryGenerator.ControlVectorList list = new TrajectoryGenerator.ControlVectorList();
        Path fullFile = Filesystem.getDeployDirectory().toPath().resolve(filename);
        try(BufferedReader reader = Files.newBufferedReader(fullFile)){

            //Read the CSV file
            reader.readLine();

            String line;
            while ((line = reader.readLine()) != null){
                StringTokenizer tokenizer = new StringTokenizer(line, ",");
                double xPath = Double.parseDouble(tokenizer.nextToken());
                double yPath = Double.parseDouble(tokenizer.nextToken());
                double xTangent = Double.parseDouble(tokenizer.nextToken());
                double yTangent = Double.parseDouble(tokenizer.nextToken());
                list.add(new Spline.ControlVector(
                    new double[]{xPath, xTangent, 0}, 
                    new double[]{yPath, yTangent, 0}));
            }

        }catch (IOException e){
            throw new RuntimeException(e);
        }

        return TrajectoryGenerator.generateTrajectory(list, trajectoryConfig);
        
    }

    public static TrajectoryConfig getDefaultTrajectoryConfig(double maxSpeedMetersPerSecond, double maxAccelerationMetersPerSecondSquared) {
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                    Constants.DriveConstants.kvVoltSecondsPerMeter,
                    Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics,
                5);

        return new TrajectoryConfig(maxSpeedMetersPerSecond,
            maxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics);
    }

    public static TrajectoryConfig getDefaultTrajectoryConfig(){
        return getDefaultTrajectoryConfig(defualtVelocity, defaultAccel);
    }

    public static TrajectoryConfig getSlowerTrajectoryConfig(){
        return getDefaultTrajectoryConfig(slowerVelocity, slowerAccel);
    }

    public static CommandBase startTrajectory(String filename, TrajectoryConfig config, Drivetrain drive){
        Trajectory traj = loadTrajectory(filename, config);
        FollowTraj followTrajectory = new FollowTraj(traj, drive);
        SetInitialOdometryCommand setOdom = new SetInitialOdometryCommand(drive, traj.getInitialPose());

        return setOdom.andThen(followTrajectory);
    }

    public static CommandBase createTrajectory(String filename, TrajectoryConfig config, Drivetrain drive){
        Trajectory traj = loadTrajectory(filename, config);
        return new FollowTraj(traj, drive);
    }
    
}
