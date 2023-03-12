package frc.robot.commands.gridcommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveBottomGrid extends CommandBase{

        //No Auto Scoring Currently, work on later

        DrivetrainSubsystem drivetrainSubsystem;
        Command pathCommand;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath(Constants.CycleTrajectoryFileNames.TOP_TO_BOTTOM_GRID, new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration), true);

        public MoveBottomGrid(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){
                double distanceFromTrajectoryStart = 
                        Math.sqrt(Math.pow(trajectory.getInitialPose().relativeTo(drivetrainSubsystem.getPose()).getX(), 2) + 
                        Math.pow(trajectory.getInitialPose().relativeTo(drivetrainSubsystem.getPose()).getY(), 2));
                if(distanceFromTrajectoryStart > Constants.DrivetrainCharacteristics.maxCycleErrorDistanceMeters) return;

                pathCommand = Autos.runCyclePath(drivetrainSubsystem, trajectory);
                pathCommand.schedule();
        }

        @Override
        public boolean isFinished(){
                return drivetrainSubsystem.autoInterrupted;
        }

        @Override
        public void end(boolean gotInterrupted){
                if(pathCommand != null) pathCommand.cancel();
        }
}
