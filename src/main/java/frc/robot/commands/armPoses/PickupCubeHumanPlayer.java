package frc.robot.commands.armPoses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;

public class PickupCubeHumanPlayer extends SequentialCommandGroup {
        ArmSubsystem subsystem;
        public PickupCubeHumanPlayer(ArmSubsystem subsystem) {
                this.subsystem = subsystem;
                addRequirements(subsystem);
                addCommands(
                        new InstantCommand(()->{
                                this.subsystem.commandAngle(125.0);
                        }),
                        new WaitUntilCommand(this.subsystem::shoulderAtTarget),
                        new InstantCommand(()->{
                                this.subsystem.ElbowCommandAngle(83.85);
                        }),
                        new WaitUntilCommand(()->false)
                );
        }
}