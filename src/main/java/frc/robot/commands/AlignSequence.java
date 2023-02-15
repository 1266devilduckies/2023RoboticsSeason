package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignSequence extends SequentialCommandGroup {
        
        public AlignSequence(DrivetrainSubsystem drivetrainSubsystem, double theta, double distance){
                addCommands(
                        //uncomment if irl

                        //new RotateToAngle(drivetrainSubsystem, drivetrainSubsystem.getCamera().getLatestResult().getBestTarget().getSkew()),
                        new RotateToAngle(drivetrainSubsystem, theta),
                        new DriveToPosition(drivetrainSubsystem, -distance),
                        new RotateToAngle(drivetrainSubsystem, -theta),
                        new DriveToPosition(drivetrainSubsystem, distance * -Math.cos(Units.radiansToDegrees(theta)))
                        //new RotateToAngle(drivetrainSubsystem, drivetrainSubsystem.getCamera().getLatestResult().getBestTarget().getSkew()) 
                );
        }

}
