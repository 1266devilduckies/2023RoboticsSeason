// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.GrabGamePiece;
import frc.robot.commands.SpitOutGamePiece;
import frc.robot.commands.armPoses.GoHome;
import frc.robot.commands.armPoses.GroundConeScore;
import frc.robot.commands.armPoses.GroundCubeScore;
import frc.robot.commands.armPoses.HighConeScore;
import frc.robot.commands.armPoses.HighCubeScore;
import frc.robot.commands.armPoses.MidConeScore;
import frc.robot.commands.armPoses.MidCubeScore;
import frc.robot.commands.armPoses.PickupCone;
import frc.robot.commands.armPoses.PickupConeHumanPlayer;
import frc.robot.commands.armPoses.PickupConeHumanPlayerRamp;
import frc.robot.commands.armPoses.PickupCube;
import frc.robot.commands.armPoses.PickupCubeHumanPlayer;
import frc.robot.commands.armPoses.PickupCubeHumanPlayerRamp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(this);
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  
  public SendableChooser<DuckAutoProfile> autonomousMode = new SendableChooser<DuckAutoProfile>();

  public final static CommandPS4Controller driverJoystick =
      new CommandPS4Controller(DriverConstants.port);
  public final static CommandXboxController operatorJoystick =
      new CommandXboxController(OperatorConstants.port);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Autos.pushAutosToDashboard(autonomousMode, drivetrainSubsystem, armSubsystem, clawSubsystem);
    Shuffleboard.getTab("autos").add(autonomousMode);
    // Configure the trigger bindings
    configureBindings();
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
        driverJoystick.R1().whileTrue(new Balance(drivetrainSubsystem));
        driverJoystick.square().whileTrue(new ParallelDeadlineGroup(
                new PickupConeHumanPlayerRamp(armSubsystem),
                new GrabGamePiece(clawSubsystem, false)));
        driverJoystick.square().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));
        driverJoystick.circle().whileTrue(new ParallelDeadlineGroup(
                new PickupCubeHumanPlayerRamp(armSubsystem),
                new GrabGamePiece(clawSubsystem, true)));
        driverJoystick.circle().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));

        //spit out both game pieces
        operatorJoystick.x().whileTrue(new SpitOutGamePiece(clawSubsystem));
        operatorJoystick.povLeft().whileTrue(new SpitOutGamePiece(clawSubsystem));
        
        //pickup cube floor
        operatorJoystick.rightTrigger(0.8).whileTrue(new ParallelDeadlineGroup(
                new GrabGamePiece(clawSubsystem, true),        
                new PickupCube(armSubsystem)
                ));
        operatorJoystick.rightTrigger(0.8).onFalse(new GoHome(armSubsystem));
        //pickup cone floor
        operatorJoystick.leftTrigger(0.8).whileTrue(new ParallelDeadlineGroup(
                new GrabGamePiece(clawSubsystem, false),      
                new PickupCone(armSubsystem)
        ));
        operatorJoystick.leftTrigger(0.8).onFalse(new GoHome(armSubsystem));

        //pickup cone from human player
        operatorJoystick.leftBumper().whileTrue(new ParallelDeadlineGroup(
                new PickupConeHumanPlayer(armSubsystem),
                new GrabGamePiece(clawSubsystem, false)));
        operatorJoystick.leftBumper().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));

        //pickup cube from human player
        operatorJoystick.rightBumper().whileTrue(new ParallelDeadlineGroup(
                new PickupCubeHumanPlayer(armSubsystem),
                new GrabGamePiece(clawSubsystem, true)));
        operatorJoystick.rightBumper().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));

        //deposit cube ground
        operatorJoystick.a().whileTrue(new GroundCubeScore(armSubsystem, clawSubsystem));
        operatorJoystick.a().onFalse(new SequentialCommandGroup(
                new InstantCommand(()->{
                        clawSubsystem.superSpeed = false;
                }),
                new WaitCommand(0.5),
                new GoHome(armSubsystem)));

        //deposit cone ground
        operatorJoystick.povDown().whileTrue(new GroundConeScore(armSubsystem));
        operatorJoystick.povDown().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)));

        //deposit cube mid
        operatorJoystick.b().whileTrue(new MidCubeScore(armSubsystem));
        operatorJoystick.b().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));

        //deposit cone mid
        operatorJoystick.povRight().whileTrue(new MidConeScore(armSubsystem));
        operatorJoystick.povRight().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));

        //deposit cube high
        operatorJoystick.y().whileTrue(new HighCubeScore(armSubsystem));
        operatorJoystick.y().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));

        //deposit cone high
        operatorJoystick.povUp().whileTrue(new HighConeScore(armSubsystem));
        operatorJoystick.povUp().onFalse(new SequentialCommandGroup(
                new WaitCommand(0.5),
                new GoHome(armSubsystem)
        ));
  }

  public DuckAutoProfile getAutonomousProfile() {
    // An example command will be run in autonomous
    return autonomousMode.getSelected();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem(){
    return drivetrainSubsystem;
  }
  public ArmSubsystem getArmSubsystem(){
        return armSubsystem;
  }
  public ClawSubsystem getClawSubsystem() {
        return clawSubsystem;
  }
  public LEDSubsystem getLEDSubsystem() {
        return ledSubsystem;
  }
}
