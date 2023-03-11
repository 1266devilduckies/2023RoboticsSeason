package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        double y = -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.ForwardDriveAxis);
        double x = RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.TurningDriveAxis);
        
        drivetrainSubsystem.robotDrive.setMaxOutput(Constants.DrivetrainCharacteristics.speedScale);
        drivetrainSubsystem.robotDrive.arcadeDrive(y,-x);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}
