package frc.robot.commands.armPoses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;

public class GoHome extends SequentialCommandGroup {
        ArmSubsystem subsystem;
        public GoHome(ArmSubsystem subsystem) {
                this.subsystem = subsystem;
                addRequirements(subsystem);
                addCommands(
                        new InstantCommand(()->{
                                this.subsystem.ElbowCommandAngle(5.0);
                        }),
                        new WaitUntilCommand(this.subsystem::elbowAtTarget),
                        new InstantCommand(()->{
                                this.subsystem.commandAngle(5.0);
                        })
                );
        }
}
