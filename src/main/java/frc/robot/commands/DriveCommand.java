package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase{
    
    DrivetrainSubsystem drivetrainSubsystem;
    public DriveCommand(DrivetrainSubsystem subsystem){
        drivetrainSubsystem = subsystem;
        addRequirements(drivetrainSubsystem);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double x = -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.ForwardDriveAxis);
        double y = RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.TurningDriveAxis);
    
        // extremize
        double tempY = y;
        y = Math.signum(x) * (Math.abs(x) < Constants.DriverConstants.deadbandLeftJoystick ? 0 : 1);
        x = Math.signum(tempY) * (Math.abs(tempY) < Constants.DriverConstants.deadbandRightJoystick ? 0 : 1);
    
        if (y != 0) {
            drivetrainSubsystem.robotDrive.arcadeDrive(y, -x);
        } else {
            drivetrainSubsystem.robotDrive.tankDrive(x, -x);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}
