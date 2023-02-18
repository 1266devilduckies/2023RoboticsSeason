package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Balance extends CommandBase{

        private DrivetrainSubsystem drivetrainSubsystem;
        public Balance(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){
                drivetrainSubsystem.pidGyroPitch.setSetpoint(0.0);
        }
        @Override
        public void execute() {
                drivetrainSubsystem.pidGyroPitch.calculate(drivetrainSubsystem.gyro.getPitch(), 0.0);
        }

        public boolean isFinished() {
                return drivetrainSubsystem.pidGyroPitch.atSetpoint();
        }

        public void end(boolean gotInterrupted) {
                
        }
}
