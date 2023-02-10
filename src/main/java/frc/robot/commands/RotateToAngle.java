package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateToAngle extends CommandBase {
        DrivetrainSubsystem m_drivetrainSubsystem;
        double m_angle;
        PIDController pidController = new PIDController(0.0493, 0.0, 0.0);
        public RotateToAngle(DrivetrainSubsystem drivetrain, double angle) {
                m_drivetrainSubsystem = drivetrain;
                m_angle = angle;
                pidController.setTolerance(0.5, 2); //0.5 degree tolerance, changing at a rate of 2 degrees per second
                addRequirements(m_drivetrainSubsystem);
        }
        
        public void initialize() {
                //left negative, right positive
                double globalRotation = m_drivetrainSubsystem.gyro.getAngle() - m_angle;
                pidController.setSetpoint(globalRotation);
        }
        public void execute() {
                double controlEffort = pidController.calculate(m_drivetrainSubsystem.gyro.getAngle());
                SmartDashboard.putNumber("error", pidController.getSetpoint() - m_drivetrainSubsystem.gyro.getAngle());
                m_drivetrainSubsystem.robotDrive.tankDrive(controlEffort + Math.signum(controlEffort)*0.2, -controlEffort - Math.signum(controlEffort)*0.2);
        }
        public boolean isFinished() {
                return pidController.atSetpoint();
        }
        public void end(boolean interrupted) {
                m_drivetrainSubsystem.tankDriveVolts(0, 0);
                SmartDashboard.putNumber("was interrupted or ended", Timer.getFPGATimestamp());
        }
}
