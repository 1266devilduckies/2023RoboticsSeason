package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignSequence extends SequentialCommandGroup{
        
        public AlignSequence(DrivetrainSubsystem drivetrainSubsystem, double theta, double distance){
                addCommands(
                        new RotateToAngle(drivetrainSubsystem, theta),
                        new DriveToPosition(drivetrainSubsystem, -distance),
                        new RotateToAngle(drivetrainSubsystem, -theta),
                        new DriveToPosition(drivetrainSubsystem, distance)
                );
        }

}
