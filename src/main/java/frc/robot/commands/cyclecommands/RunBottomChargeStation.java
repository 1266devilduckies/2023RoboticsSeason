package frc.robot.commands.cyclecommands;

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

public class RunBottomChargeStation extends CommandBase{
     
        //Assumes that robot is starting inside loading zone

        DrivetrainSubsystem drivetrainSubsystem;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("LoadingToBottomChargeStation", new PathConstraints(
                Constants.DrivetrainCharacteristics.cycleSpeed,
              Constants.DrivetrainCharacteristics.cycleAcceleration), true);

        public RunBottomChargeStation(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){
                Autos.runCyclePath(drivetrainSubsystem, trajectory).schedule();
        }
}
