package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Balance extends CommandBase{

        private DrivetrainSubsystem drivetrainSubsystem;
        private double initOffset;
        private final double offsetFromReset = 100000;
        private double goalLeft;
        private double goalRight;
        PIDController leftPID = new PIDController(Constants.DrivetrainCharacteristics.rampPGain, 0.0, 0.0);
        PIDController rightPID = new PIDController(Constants.DrivetrainCharacteristics.rampPGain, 0.0, 0.0);
        private double maxSpeed = 0.4;
        public Balance(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){

                drivetrainSubsystem.MainLeftMotorBack.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainRightMotorBack.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainLeftMotorFront.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainRightMotorFront.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.leftTopMotor.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.rightTopMotor.setNeutralMode(NeutralMode.Coast);

                goalLeft = drivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition() + offsetFromReset;
                goalRight = drivetrainSubsystem.MainRightMotorBack.getSelectedSensorPosition() + offsetFromReset;
                // drivetrainSubsystem.MainLeftMotorBack.config_kP(0, Constants.DrivetrainCharacteristics.rampPGain);
                // drivetrainSubsystem.MainRightMotorBack.config_kP(0, Constants.DrivetrainCharacteristics.rampPGain);
                
        }
        @Override
        public void execute() {
                double leftOutput = leftPID.calculate(drivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition(), goalLeft);
                double rightOutput = rightPID.calculate(drivetrainSubsystem.MainRightMotorBack.getSelectedSensorPosition(), goalRight);
                drivetrainSubsystem.MainLeftMotorBack.set(ControlMode.PercentOutput, MathUtil.clamp(leftOutput, -maxSpeed, maxSpeed));
                drivetrainSubsystem.MainRightMotorBack.set(ControlMode.PercentOutput, MathUtil.clamp(rightOutput, -maxSpeed, maxSpeed));
                // drivetrainSubsystem.MainLeftMotorBack.set(ControlMode.Position, goalLeft);
                // drivetrainSubsystem.MainRightMotorBack.set(ControlMode.Position, goalRight);
        }

        public boolean isFinished() {
                return false;
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
