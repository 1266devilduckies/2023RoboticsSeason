// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

  //example auto that drives forward 
  public static CommandBase runPath(DrivetrainSubsystem drivetrainSubsystem, AutoPath autoPath) {
    PathPlannerTrajectory path = PathPlanner.loadPath(autoPath.getPathName(), new PathConstraints(4, 3)); //in terms of m/s
    
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    drivetrainSubsystem::getPose, 
    drivetrainSubsystem::resetOdometry, 
    drivetrainSubsystem.ramseteController, 
    drivetrainSubsystem.drivetrainKinematics, 
    drivetrainSubsystem::tankDriveMetersPerSecond, 
    Constants.eventMap, 
    drivetrainSubsystem);

    return autoBuilder.fullAuto(path);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
