package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.DuckGearUtil;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPosition extends CommandBase {
        DrivetrainSubsystem drivetrainSubsystem;
        double leftPosition;
        double rightPosition;
        double relativePositionTicks;
        public DriveToPosition(DrivetrainSubsystem drivetrain, double relativePositionMeters) {
                relativePositionTicks = DuckGearUtil.metersToEncoderTicks(
                        relativePositionMeters, 
                        Constants.DrivetrainCharacteristics.gearing, 
                        2048.0, 
                        Constants.DrivetrainCharacteristics.wheelRadiusMeters);

                drivetrainSubsystem = drivetrain;
                addRequirements(drivetrainSubsystem);
        }

        public void initialize(){
                leftPosition = drivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition() + relativePositionTicks;
                rightPosition = drivetrainSubsystem.MainRightMotorBack.getSelectedSensorPosition() + relativePositionTicks;
        }

        public void execute() {
                drivetrainSubsystem.MainLeftMotorBack.set(ControlMode.Position, leftPosition);
                drivetrainSubsystem.MainRightMotorBack.set(ControlMode.Position, rightPosition);
        }

        public boolean isFinished() {
                return Math.abs(drivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition() - leftPosition) < 500.0;
        }

        public void end(boolean gotInterrupted) {
                drivetrainSubsystem.tankDriveVolts(0, 0);
        }
}
