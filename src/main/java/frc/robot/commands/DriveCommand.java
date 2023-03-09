package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
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
        //drivetrainSubsystem.robotDrive.arcadeDrive(y,-x);

        double xSpeed = MathUtil.applyDeadband(y, Constants.DrivetrainCharacteristics.deadband);
        double zRotation = MathUtil.applyDeadband(x, Constants.DrivetrainCharacteristics.deadband);

        WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);

        drivetrainSubsystem.encoderBasedDrive(wheelSpeeds.left, wheelSpeeds.right);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}
