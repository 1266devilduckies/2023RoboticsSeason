package frc.robot.commands.cyclecommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunTopChargeStation extends CommandBase{
        
        //Assumes that robot is starting inside loading zone

        DrivetrainSubsystem drivetrainSubsystem;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("LoadingToTopChargeStation", new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration), true);

        public RunTopChargeStation(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){
                Autos.runCyclePath(drivetrainSubsystem, trajectory).schedule();
        }
}
