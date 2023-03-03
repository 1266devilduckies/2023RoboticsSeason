package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
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
        
        @Override
        public void initialize() {
                m_drivetrainSubsystem.pidGlobalRotation.setTolerance(3, 2); //3                                                                   rm$e degree tolerance, changing at a rate of 2 degrees per second
                m_drivetrainSubsystem.pidGlobalRotation.setSetpoint(m_angle);
        }

        @Override
        public void execute() {
                double odometryRotation = m_drivetrainSubsystem.getPose().getRotation().getDegrees();
                odometryRotation = odometryRotation % 360.;
                double controlEffort = m_drivetrainSubsystem.pidGlobalRotation.calculate(odometryRotation, m_angle);
                controlEffort = 1;//MathUtil.clamp(controlEffort, -0.5, 0.5);
                SmartDashboard.putNumber("error", m_drivetrainSubsystem.pidGlobalRotation.getPositionError());
                m_drivetrainSubsystem.robotDrive.setMaxOutput(1);
                //m_drivetrainSubsystem.robotDrive.tankDrive(/*controlEffort + */Math.signum(controlEffort)*Constants.DrivetrainCharacteristics.kSAngular, /*-controlEffort*/ - Math.signum(controlEffort)*Constants.DrivetrainCharacteristics.kSAngular);
                
                m_drivetrainSubsystem.MainLeftMotorBack.set(ControlMode.PercentOutput, Constants.DrivetrainCharacteristics.kSAngular);
                m_drivetrainSubsystem.MainRightMotorBack.set(ControlMode.PercentOutput, -Constants.DrivetrainCharacteristics.kSAngular);


                SmartDashboard.putNumber("tankDrive", Math.signum(controlEffort)*Constants.DrivetrainCharacteristics.kSAngular);
        }

        @Override
        public boolean isFinished() {
                return m_drivetrainSubsystem.pidGlobalRotation.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
                m_drivetrainSubsystem.tankDriveVolts(0, 0);
                SmartDashboard.putNumber("was interrupted or ended", Timer.getFPGATimestamp());
        }
}
