package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPosition extends CommandBase {
        DrivetrainSubsystem drivetrainSubsystem;
        double leftPosition;
        double rightPosition;
        public DriveToPosition(DrivetrainSubsystem drivetrain, double relativePosition) {
                leftPosition = drivetrain.MainLeftMotorBack.getSelectedSensorPosition() + relativePosition;
                rightPosition = drivetrain.MainRightMotorBack.getSelectedSensorPosition() + relativePosition;
                drivetrainSubsystem = drivetrain;
                addRequirements(drivetrainSubsystem);
        }

        public void execute() {
                drivetrainSubsystem.MainLeftMotorBack.set(ControlMode.Position, leftPosition);
                drivetrainSubsystem.MainRightMotorBack.set(ControlMode.Position, rightPosition);
        }

        public boolean isFinished() {
                return Math.abs(drivetrainSubsystem.MainLeftMotorBack.getClosedLoopTarget() - drivetrainSubsystem.MainLeftMotorBack.getClosedLoopError()) < 500 &&
                Math.abs(drivetrainSubsystem.MainRightMotorBack.getClosedLoopTarget() - drivetrainSubsystem.MainRightMotorBack.getClosedLoopError()) < 500;
        }

        public void end(boolean gotInterrupted) {
                drivetrainSubsystem.tankDriveVolts(0, 0);
        }
}
