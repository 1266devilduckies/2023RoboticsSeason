package frc.robot.commands.armPoses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class GroundCubeScore extends SequentialCommandGroup {
        ArmSubsystem subsystem;
        ClawSubsystem claw;
        public GroundCubeScore(ArmSubsystem subsystem, ClawSubsystem clawSubsystem) {
                this.subsystem = subsystem;
                this.claw = clawSubsystem;
                addRequirements(subsystem);
                addCommands(
                        /*new InstantCommand(()->{
                                this.subsystem.commandAngle(10.0);
                        }),
                        new WaitUntilCommand(this.subsystem::shoulderAtTarget),*/
                        new InstantCommand(()->{
                                this.claw.superSpeed = true;
                                this.subsystem.ElbowCommandAngle(115.0);
                        }),
                        new WaitUntilCommand(()->false)
                );
        }
}