package frc.robot.commands.cyclecommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunTopChargeStation extends CommandBase{
        
        //Assumes that robot is starting inside loading zone

        DrivetrainSubsystem drivetrainSubsystem;
        Command pathCommand;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath(Constants.CycleTrajectoryFileNames.LOAD_TO_TOP, new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration), true);

        public RunTopChargeStation(DrivetrainSubsystem drivetrainSubsystem){
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
