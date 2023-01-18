// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import frc.robot.betterpathplanner.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.DuckAutoProfile;
import frc.robot.RobotContainer;
import frc.robot.betterpathplanner.PathPlannerTrajectory;
import frc.robot.betterpathplanner.RamseteAutoBuilder;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {
  
  /*public static CommandBase templateAuto(DrivetrainSubsystem drivetrainSubsystem) {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3)); //in terms of m/s
    
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    drivetrainSubsystem::getPose, 
    drivetrainSubsystem::resetOdometry, 
    drivetrainSubsystem.ramseteController, 
    drivetrainSubsystem.drivetrainKinematics, 
    drivetrainSubsystem::tankDriveMetersPerSecond, 
    Constants.eventMap,  
    drivetrainSubsystem);

    return autoBuilder.fullAuto(examplePath);
  }*/

  public static DuckAutoProfile lowDockLowAuto(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.LOW_DOCKLOW;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(4, 3));//in terms of m/s
    SequentialCommandGroup command = new SequentialCommandGroup(runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }

  public static DuckAutoProfile forwardAuto(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.LOW_FORWARD;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(4, 3));//in terms of m/s
    SequentialCommandGroup command = new SequentialCommandGroup(runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();

    return new DuckAutoProfile(command, startPosition);
  }
  //run an auto path
  private static CommandBase runPath(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory pathTrajectory) {
    //PathPlannerTrajectory path = PathPlanner.loadPath(autoPathName, new PathConstraints(4, 3)); //in terms of m/s
    
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    drivetrainSubsystem::getPose, 
    drivetrainSubsystem::resetOdometry, 
    drivetrainSubsystem.ramseteController, 
    drivetrainSubsystem.drivetrainKinematics, 
    new SimpleMotorFeedforward(Constants.DrivetrainCharacteristics.kS, Constants.DrivetrainCharacteristics.kV,Constants.DrivetrainCharacteristics.kA),
    drivetrainSubsystem::getWheelSpeeds, 
    new PIDConstants(Constants.DrivetrainCharacteristics.kP, 0.0, 0.0),
    drivetrainSubsystem::tankDriveVolts,
    Constants.eventMap,  
    true,
    drivetrainSubsystem);
    return autoBuilder.fullAuto(pathTrajectory);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
