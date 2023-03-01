package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateToAngle extends CommandBase {
        DrivetrainSubsystem m_drivetrainSubsystem;
        double m_angle;

        public RotateToAngle(DrivetrainSubsystem drivetrain, double angle) {
                m_drivetrainSubsystem = drivetrain;
                m_angle = angle;
                addRequirements(m_drivetrainSubsystem);
        }
        
        public void initialize() {
                m_drivetrainSubsystem.pidGlobalRotation.setTolerance(3, 2); //3                                                                   rm$e degree tolerance, changing at a rate of 2 degrees per second
                m_drivetrainSubsystem.pidGlobalRotation.setSetpoint(m_angle);
        }
        public void execute() {
                double controlEffort = m_drivetrainSubsystem.pidGlobalRotation.calculate(m_drivetrainSubsystem.gyro.getAngle(), m_angle);
                SmartDashboard.putNumber("error", m_drivetrainSubsystem.pidGlobalRotation.getPositionError());
                m_drivetrainSubsystem.robotDrive.tankDrive(controlEffort + Math.signum(controlEffort)*Constants.DrivetrainCharacteristics.kSAngular, -controlEffort - Math.signum(controlEffort)*Constants.DrivetrainCharacteristics.kSAngular);
        }
        public boolean isFinished() {
                return m_drivetrainSubsystem.pidGlobalRotation.atSetpoint();
        }
        public void end(boolean interrupted) {
                m_drivetrainSubsystem.tankDriveVolts(0, 0);
                SmartDashboard.putNumber("was interrupted or ended", Timer.getFPGATimestamp());
        }
}
