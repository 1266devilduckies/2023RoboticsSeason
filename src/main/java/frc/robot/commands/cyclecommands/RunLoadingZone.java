package frc.robot.commands.cyclecommands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.DuckAutoProfile;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunLoadingZone extends CommandBase{
        
        //Either starting in top or bottom of charge station in community

        DrivetrainSubsystem drivetrainSubsystem;
        List<Pose2d> poses = new ArrayList<>();
        List<PathPlannerTrajectory> possibleTrajectories = new ArrayList<>();

        //add something that decides wether the robot is near the top or bottom then chooses path accordingly

        PathPlannerTrajectory trajectoryFromTop = PathPlanner.loadPath("TopChargeStationToLoading", new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration));
        
        PathPlannerTrajectory trajectoryFromBottom = PathPlanner.loadPath("BottomChargeStationToLoading", new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration));

        public RunLoadingZone(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){
                poses.add(trajectoryFromTop.getInitialPose());
                poses.add(trajectoryFromBottom.getInitialPose());

                possibleTrajectories.add(trajectoryFromBottom);
                possibleTrajectories.add(trajectoryFromTop);

                Pose2d nearestPose = drivetrainSubsystem.getPose().nearest(poses);

                for (PathPlannerTrajectory trajectory : possibleTrajectories) {
                        if(trajectory.getInitialPose().equals(nearestPose)){
                                Autos.runCyclePath(drivetrainSubsystem, trajectory).schedule();
                                return;
                        }
                }
        }
}
