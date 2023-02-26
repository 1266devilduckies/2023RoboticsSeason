package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceComplexCommand extends SequentialCommandGroup {

        public BalanceComplexCommand(DrivetrainSubsystem drivetrainSubsystem, boolean reversed){
                addCommands(
                        new DriveToPosition(drivetrainSubsystem, 2 * (reversed ? -1 : 1)), //2
                        new Balance(drivetrainSubsystem)
                );
        }
        
}