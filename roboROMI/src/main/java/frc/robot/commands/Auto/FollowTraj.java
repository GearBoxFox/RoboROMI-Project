package frc.robot.commands.Auto;

import org.snobotv2.coordinate_gui.commands.BaseRamseteCoordinateGuiCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowTraj extends BaseRamseteCoordinateGuiCommand {
    Drivetrain drive;
    Trajectory traj;

    public FollowTraj(Trajectory traj, Drivetrain drive){
        super(traj, new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta), Constants.DriveConstants.kDriveKinematics, drive);

        this.drive = drive;
        this.traj = traj;
        
    }

    @Override
    protected Pose2d getPose() {
        // TODO Auto-generated method stub
        return drive.getPose();
    }

    @Override
    protected void setVelocity(double leftVelocityMps, double rightVelocityMps, double leftAccelMpss,
            double rightAccelMpss) {
        // TODO Auto-generated method stub
        drive.smartVelocityControl(leftVelocityMps, rightVelocityMps, leftAccelMpss, rightAccelMpss);
    }

    @Override
    protected DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {
        // TODO Auto-generated method stub
        return drive.getWheelSpeeds();
    }}
