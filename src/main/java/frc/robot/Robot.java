// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;

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
  private DoubleEntry waitDelayEntry;
  private double waitDelay = 0.0;
  private DoubleEntry velocityAutoEntry;
  private DoubleEntry accelerationAutoEntry;
  private BooleanEntry currentLimitDriveEntry;
  private PowerDistribution powerBoard;
  private PneumaticHub pneumaticHub;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    powerBoard = new PowerDistribution(1, ModuleType.kRev);
    powerBoard.clearStickyFaults();
    pneumaticHub = new PneumaticHub(9);
    pneumaticHub.clearStickyFaults();
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic kPTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/kP"));
    kPEntry = kPTopic.getEntry(0.0);
    kPEntry.setDefault(Constants.DrivetrainCharacteristics.kP);
    DoubleTopic kSTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/kS"));
    kSEntry = kSTopic.getEntry(0.0);
    DoubleTopic kVTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/kV"));
    kVEntry = kVTopic.getEntry(0.0);
    DoubleTopic kDTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/kD"));
    kDEntry = kDTopic.getEntry(0.0);
    DoubleTopic kATopic = new DoubleTopic(nt.getTopic("/SmartDashboard/kA"));
    kAEntry = kATopic.getEntry(0.0);
    kSEntry.setDefault(Constants.DrivetrainCharacteristics.kS);
    kVEntry.setDefault(Constants.DrivetrainCharacteristics.kV);
    kDEntry.setDefault(Constants.DrivetrainCharacteristics.kD);
    kAEntry.setDefault(Constants.DrivetrainCharacteristics.kA);
    DoubleTopic waitDelayTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/Wait Delay (Seconds)"));
    waitDelayEntry = waitDelayTopic.getEntry(0.0);
    waitDelayEntry.setDefault(waitDelay);
    DoubleTopic velocityAutoTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/Velocity Auto ms"));
    velocityAutoEntry = velocityAutoTopic.getEntry(0.0);
    velocityAutoEntry.setDefault(Constants.DrivetrainCharacteristics.maxAutoVelocityMeters);
    DoubleTopic accelerationAutoTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/Acceleration Auto ms2"));
    accelerationAutoEntry = accelerationAutoTopic.getEntry(0.0);
    accelerationAutoEntry.setDefault(Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters);
    BooleanTopic enableCurrentLimitingDrivetrainTopic = new BooleanTopic(nt.getTopic("/SmartDashboard/Enable Current Limiting On Drivetrain"));
    currentLimitDriveEntry = enableCurrentLimitingDrivetrainTopic.getEntry(true);
    currentLimitDriveEntry.setDefault(true);
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
    //SmartDashboard.putNumber("0.478 metersPerSecond target",
    //m_robotContainer.getDrivetrainSubsystem().getWheelSpeeds().leftMetersPerSecond);

    if (waitDelayEntry.get() != waitDelay) {
      waitDelay = waitDelayEntry.get();
      
      Autos.pushAutosToDashboard(m_robotContainer.autonomousMode, m_robotContainer.getDrivetrainSubsystem(), m_robotContainer.getClawSubsystem());
    }

    m_robotContainer.getArmSubsystem().loadPreferences();
    m_robotContainer.getDrivetrainSubsystem().loadPreferences();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.getDrivetrainSubsystem().aprilTagFieldLayout.setOrigin(
                    DriverStation.getAlliance() == Alliance.Red
                            ? OriginPosition.kRedAllianceWallRightSide
                            : OriginPosition.kBlueAllianceWallRightSide);

    DuckAutoProfile autoProfile = m_robotContainer.getAutonomousProfile();
    autoProfile.addDelay(waitDelay);
    m_autonomousCommand = autoProfile.getAutoCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_robotContainer.getDrivetrainSubsystem().resetOdometry(autoProfile.getStartingPose());
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
