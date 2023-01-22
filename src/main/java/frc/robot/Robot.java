// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private DoubleEntry kPEntry;
  private DoubleEntry kSEntry;
  private DoubleEntry kVEntry;
  private DoubleEntry kDEntry;
  private DoubleEntry kAEntry;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DoubleTopic kPTopic = new DoubleTopic(NetworkTableInstance.getDefault().getTopic("/SmartDashboard/kP"));
    kPEntry = kPTopic.getEntry(0.0);
    kPEntry.setDefault(Constants.DrivetrainCharacteristics.kP);
    DoubleTopic kSTopic = new DoubleTopic(NetworkTableInstance.getDefault().getTopic("/SmartDashboard/kS"));
    kSEntry = kSTopic.getEntry(0.0);
    DoubleTopic kVTopic = new DoubleTopic(NetworkTableInstance.getDefault().getTopic("/SmartDashboard/kV"));
    kVEntry = kVTopic.getEntry(0.0);
    DoubleTopic kDTopic = new DoubleTopic(NetworkTableInstance.getDefault().getTopic("/SmartDashboard/kD"));
    kDEntry = kDTopic.getEntry(0.0);
    DoubleTopic kATopic = new DoubleTopic(NetworkTableInstance.getDefault().getTopic("/SmartDashboard/kA"));
    kAEntry = kATopic.getEntry(0.0);
    kSEntry.setDefault(Constants.DrivetrainCharacteristics.kS);
    kVEntry.setDefault(Constants.DrivetrainCharacteristics.kV);
    kDEntry.setDefault(Constants.DrivetrainCharacteristics.kD);
    kAEntry.setDefault(Constants.DrivetrainCharacteristics.kA);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("0.478 metersPerSecond target",
    m_robotContainer.getDrivetrainSubsystem().getWheelSpeeds().leftMetersPerSecond);

    if (kSEntry.get() != Constants.DrivetrainCharacteristics.kS ||
    kVEntry.get() != Constants.DrivetrainCharacteristics.kV ||
    kPEntry.get() != Constants.DrivetrainCharacteristics.kP ||
    kDEntry.get() != Constants.DrivetrainCharacteristics.kD ||
    kAEntry.get() != Constants.DrivetrainCharacteristics.kA) {
      Constants.DrivetrainCharacteristics.kS = kSEntry.get();
      Constants.DrivetrainCharacteristics.kV = kVEntry.get();
      Constants.DrivetrainCharacteristics.kP = kPEntry.get();
      Constants.DrivetrainCharacteristics.kD = kDEntry.get();
      Constants.DrivetrainCharacteristics.kA = kAEntry.get();
      m_robotContainer.autonomousMode.addOption("forward auto", Autos.forwardAuto(m_robotContainer.getDrivetrainSubsystem()));
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousProfile().getAutoCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_robotContainer.getDrivetrainSubsystem().resetOdometry(m_robotContainer.getAutonomousProfile().getStartingPose());
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDrivetrainSubsystem().setMotorsToCoast();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
