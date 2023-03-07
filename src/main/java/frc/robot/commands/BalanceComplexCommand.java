package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceComplexCommand extends SequentialCommandGroup {

        public BalanceComplexCommand(DrivetrainSubsystem drivetrainSubsystem){
                addCommands(
                        new DriveToPosition(drivetrainSubsystem),
                        new Balance(drivetrainSubsystem)
                );
        }
        
}