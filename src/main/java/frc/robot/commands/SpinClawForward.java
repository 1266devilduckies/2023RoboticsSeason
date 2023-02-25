package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class SpinClawForward extends CommandBase {
        ClawSubsystem clawSubsystem;
        public SpinClawForward(ClawSubsystem clawSubsystem) {
                this.clawSubsystem = clawSubsystem;
                addRequirements(this.clawSubsystem);
        }
        public void execute() {
                double controlEffort = Preferences.getDouble(Constants.ClawCharacteristics.clawSpeedKey, this.clawSubsystem.getClawSpeed());
                this.clawSubsystem.motor.set(ControlMode.PercentOutput, controlEffort);
        }

        public boolean isFinished() {
                return true;
        }

        @Override
        public void end(boolean interrupted) {
                
        }
}
