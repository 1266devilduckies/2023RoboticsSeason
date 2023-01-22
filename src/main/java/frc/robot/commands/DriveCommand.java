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
        //kS xor kV must be 0
        double voltage = 5;//Constants.DrivetrainCharacteristics.kS + Constants.DrivetrainCharacteristics.kV;
        if (y != 0) {
            double left = y + x;
            double right = y - x;
            drivetrainSubsystem.tankDriveVolts(
                Math.signum(left) * voltage,
                Math.signum(right) * voltage
            );
        } else {
            drivetrainSubsystem.tankDriveVolts(
                Math.signum(x) * voltage,
                Math.signum(-x) * voltage
            );
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}
