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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.DuckAutoProfile;
<<<<<<< HEAD
import frc.robot.commands.armPoses.GoHome;
import frc.robot.commands.armPoses.HighCubeScore;
import frc.robot.commands.armPoses.MidConeScore;
import frc.robot.commands.armPoses.PickupCube;
=======
>>>>>>> 182a43ba894cef8dc759a99f63f8209db8c86769
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  private static DuckAutoProfile highTaxiPath(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.HIGH_TAXI;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
      Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  } 
  private static DuckAutoProfile lowTaxiPath(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.LOW_TAXI;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
    Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }


  public static DuckAutoProfile midBalance(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.MID_BALANCE;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
      Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters), true);
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory), 
        new WaitCommand(1), 
        new InstantCommand(()->{
          drivetrainSubsystem.gyro.resetPitch();
          drivetrainSubsystem.MainLeftMotorBack.setSelectedSensorPosition(0);
          drivetrainSubsystem.MainRightMotorBack.setSelectedSensorPosition(0);
        })
        ,new BalanceComplexCommand(drivetrainSubsystem)
      );
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }

  private static DuckAutoProfile justBalance(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.MID_BALANCE;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      1, 
    0.75), true);
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(new BalanceComplexCommand(drivetrainSubsystem), startPosition);
  }
  private static DuckAutoProfile demo(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    CommandBase command = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
                new GrabGamePiece(clawSubsystem, true),        
                new PickupCube(armSubsystem)
      ),
      new WaitCommand(1), //wait for the person feeding the cube to move as to not hurt them
      new HighCubeScore(armSubsystem),
      new WaitCommand(5), //hold for 5 seconds
      new ParallelDeadlineGroup(
                    new WaitCommand(0.2),
                    new SpitOutGamePiece(clawSubsystem)
      ),
      new ParallelDeadlineGroup(
              new GrabGamePiece(clawSubsystem, true),        
              new PickupCube(armSubsystem)
      ),
      new WaitCommand(1), //wait for the person feeding the cube to move as to not hurt them,
      new MidConeScore(armSubsystem),
      new WaitCommand(5), //hold for 5 seconds
      new ParallelDeadlineGroup(
                    new WaitCommand(0.2),
                    new SpitOutGamePiece(clawSubsystem)
      ) //do not wait as somebody might move in that timeframe if it is not moving
    );
    return new DuckAutoProfile(command.repeatedly(), new Pose2d());
  }

  //auto factory

  //changed to public static, if problem arises change back to private static
  public static CommandBase runPath(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory pathTrajectory) {
    
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
    return autoBuilder.fullAuto(pathTrajectory).andThen(() -> drivetrainSubsystem.tankDriveVolts(0,0));
  }

  public static CommandBase runCyclePath(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory pathTrajectory) {
    
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
        return autoBuilder.followPath(pathTrajectory).andThen(() -> drivetrainSubsystem.tankDriveVolts(0,0));
   }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static void pushAutosToDashboard(SendableChooser<DuckAutoProfile> autonomousMode,
      DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {

        autonomousMode.setDefaultOption("low taxi auto", lowTaxiPath(drivetrainSubsystem));
    autonomousMode.addOption("mid balance auto", midBalance(drivetrainSubsystem));
    autonomousMode.addOption("high taxi auto", highTaxiPath(drivetrainSubsystem));
    autonomousMode.addOption("do nothing", new DuckAutoProfile());
    autonomousMode.addOption("just balance auto", justBalance(drivetrainSubsystem));
    autonomousMode.addOption("score mid balance", scoreAndMidBalance(drivetrainSubsystem, armSubsystem, clawSubsystem));
    autonomousMode.addOption("score low taxi", scoreAndLowTaxi(drivetrainSubsystem, armSubsystem, clawSubsystem));
    autonomousMode.addOption("demo", demo(armSubsystem, clawSubsystem));
  }
}