// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.DuckAutoProfile;
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

  public static DuckAutoProfile forwardAuto(DrivetrainSubsystem drivetrainSubsystem){
    SequentialCommandGroup command = new SequentialCommandGroup(runPath(drivetrainSubsystem, Constants.AutoTrajectoryFileNames.LOW_FORWARD));
    Pose2d startPosition = new Pose2d(1.81,0.94, Rotation2d.fromRadians(-0.34)); //1.81,0.94
    return new DuckAutoProfile(command, startPosition);
  }
  //run an auto path
  private static CommandBase runPath(DrivetrainSubsystem drivetrainSubsystem, String autoPathName) {
    PathPlannerTrajectory path = PathPlanner.loadPath(autoPathName, new PathConstraints(4, 3)); //in terms of m/s
    
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
    drivetrainSubsystem);
    return autoBuilder.fullAuto(path);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
