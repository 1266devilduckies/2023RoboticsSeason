package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPosition extends CommandBase {
        DrivetrainSubsystem drivetrainSubsystem;
        double originalPitch;
        boolean startedGoingUp = false;
        public DriveToPosition(DrivetrainSubsystem drivetrain) {
                drivetrainSubsystem = drivetrain;
                addRequirements(drivetrainSubsystem);
        }

        public void initialize(){
                drivetrainSubsystem.MainLeftMotorBack.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainRightMotorBack.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainLeftMotorFront.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainRightMotorFront.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.leftTopMotor.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.rightTopMotor.setNeutralMode(NeutralMode.Coast);
                originalPitch = drivetrainSubsystem.gyro.getPitch();
                startedGoingUp = false;
        }

        public void execute() {
                if (drivetrainSubsystem.gyro.getPitch()-originalPitch > 11.5 && !startedGoingUp) {
                        startedGoingUp = true;
                }
                drivetrainSubsystem.robotDrive.arcadeDrive(.8, 0.);
        }

        public boolean isFinished() {
                return  startedGoingUp;/*&& drivetrainSubsystem.gyro.getPitch()-originalPitch < 6.0;*/
        }

        public void end(boolean gotInterrupted) {
                drivetrainSubsystem.MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainLeftMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.leftTopMotor.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.rightTopMotor.setNeutralMode(NeutralMode.Brake);
        }
}