// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MotorTestSubsystem;

import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final MotorTestSubsystem motorTestSubsystem = new MotorTestSubsystem();
  private SendableChooser<DuckAutoProfile> autonomousMode = new SendableChooser<DuckAutoProfile>();

  private CommandScheduler commandScheduler = CommandScheduler.getInstance();

  public final static CommandPS4Controller driverJoystick =
      new CommandPS4Controller(DriverConstants.port);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DuckAutoProfile emptyProfile = new DuckAutoProfile(this);
    autonomousMode.setDefaultOption("Do nothing", emptyProfile);
    autonomousMode.addOption("forward auto", Autos.forwardAuto(drivetrainSubsystem));

    SmartDashboard.putData(autonomousMode);
    // Configure the trigger bindings
    configureBindings();
    //configure the auton markers
    configureMarkers();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // driverJoystick.square().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  private void configureMarkers() {
    Constants.eventMap.put("marker1", new PrintCommand("passed marker 1"));
  }

  public DuckAutoProfile getAutonomousProfile() {
    // An example command will be run in autonomous
    return autonomousMode.getSelected();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem(){
    return drivetrainSubsystem;
  }
}
