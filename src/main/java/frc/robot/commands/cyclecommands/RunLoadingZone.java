package frc.robot.commands.cyclecommands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunLoadingZone extends CommandBase{
        
        //Either starting in top or bottom of charge station in community

        DrivetrainSubsystem drivetrainSubsystem;
        List<Pose2d> poses = new ArrayList<>();
        List<PathPlannerTrajectory> possibleTrajectories = new ArrayList<>();

        Command pathCommand;

        //add something that decides wether the robot is near the top or bottom then chooses path accordingly

        PathPlannerTrajectory trajectoryFromTop = PathPlanner.loadPath(Constants.CycleTrajectoryFileNames.TOP_TO_LOAD, new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration));
        
        PathPlannerTrajectory trajectoryFromBottom = PathPlanner.loadPath(Constants.CycleTrajectoryFileNames.BOTTOM_TO_LOAD, new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration));

              PathPlannerTrajectory trajectoryFromMiddle= PathPlanner.loadPath(Constants.CycleTrajectoryFileNames.MID_TO_LOAD, new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration));

        public RunLoadingZone(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){
                poses.add(trajectoryFromTop.getInitialPose());
                poses.add(trajectoryFromBottom.getInitialPose());
                poses.add(trajectoryFromMiddle.getInitialPose());

                possibleTrajectories.add(trajectoryFromBottom);
                possibleTrajectories.add(trajectoryFromTop);
                possibleTrajectories.add(trajectoryFromMiddle);

                Pose2d nearestPose = drivetrainSubsystem.getPose().nearest(poses);

                for (PathPlannerTrajectory trajectory : possibleTrajectories) {
                        if(trajectory.getInitialPose().equals(nearestPose)){
                                double distanceFromTrajectoryStart = 
                                        Math.sqrt(Math.pow(trajectory.getInitialPose().relativeTo(drivetrainSubsystem.getPose()).getX(), 2) + 
                                        Math.pow(trajectory.getInitialPose().relativeTo(drivetrainSubsystem.getPose()).getY(), 2));
                                if(distanceFromTrajectoryStart > Constants.DrivetrainCharacteristics.maxCycleErrorDistanceMeters) return;

                                pathCommand = Autos.runCyclePath(drivetrainSubsystem, trajectory);
                                pathCommand.schedule();
                                return;
                        }
                }
        }

        @Override
        public boolean isFinished(){
                SmartDashboard.putBoolean("autoInterrupted", drivetrainSubsystem.autoInterrupted);
                return drivetrainSubsystem.autoInterrupted;
        }

        @Override
        public void end(boolean gotInterrupted){
                if(pathCommand != null) pathCommand.cancel();
        }
}
