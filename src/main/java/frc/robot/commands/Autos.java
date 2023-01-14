// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {
  
public Auto()

  public enum Auto{
    HIGH_DOCKHIGH,
    HIGH_DOCKLOW,
    HIGH_FORWARD(),
  }

  public static CommandBase templateAuto(DrivetrainSubsystem drivetrainSubsystem) {
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
  }

  //example auto that drives forward 
  public static CommandBase driveForwardAuto(DrivetrainSubsystem drivetrainSubsystem) {
    PathPlannerTrajectory path = PathPlanner.loadPath("ForwardPath", new PathConstraints(4, 3)); //in terms of m/s
    
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
