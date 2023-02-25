// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.DuckAutoProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  private static DuckAutoProfile midDockPath(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.MID_DOCK;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
      Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }  
  private static DuckAutoProfile lowDockPath(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.LOW_DOCK;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
    Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }

  /*private static DuckAutoProfile forwardAuto(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.LOW_FORWARD;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
    Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();

    return new DuckAutoProfile(command, startPosition);
  }*/

  //auto factory
  private static CommandBase runPath(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory pathTrajectory) {
    
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    drivetrainSubsystem::getPose, 
    drivetrainSubsystem::resetOdometry, 
    drivetrainSubsystem.ramseteController, 
    drivetrainSubsystem.drivetrainKinematics, 
    new SimpleMotorFeedforward(Constants.DrivetrainCharacteristics.kS, Constants.DrivetrainCharacteristics.kV,Constants.DrivetrainCharacteristics.kA),
    drivetrainSubsystem::getWheelSpeeds, 
    new PIDConstants(Constants.DrivetrainCharacteristics.kP, 0.0, Constants.DrivetrainCharacteristics.kD),
    drivetrainSubsystem::tankDriveVolts,
    Constants.eventMap,  
    true,
    drivetrainSubsystem);
    return autoBuilder.fullAuto(pathTrajectory).andThen(() -> drivetrainSubsystem.tankDriveVolts(0,0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static void pushAutosToDashboard(SendableChooser<DuckAutoProfile> autonomousMode,
      DrivetrainSubsystem drivetrainSubsystem) {

    autonomousMode.setDefaultOption("Do nothing", new DuckAutoProfile());
    
    //autonomousMode.addOption("forward auto", forwardAuto(drivetrainSubsystem));
    autonomousMode.addOption("low dock auto", lowDockPath(drivetrainSubsystem));
    autonomousMode.addOption("mid dock auto", midDockPath(drivetrainSubsystem));
  }
}
