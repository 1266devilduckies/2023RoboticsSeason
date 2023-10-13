package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawHoldPosition extends CommandBase {

        ClawSubsystem clawSubsystem;
        public ClawHoldPosition(ClawSubsystem clawSubsystem) {
                this.clawSubsystem = clawSubsystem;
                addRequirements(this.clawSubsystem);
        }
        public void initialize() {
              
        }
        public void execute() {
                 this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.05 * (this.clawSubsystem.suckingInCube == true ? -1 : 1));
        }

        @Override
        public boolean isFinished() {
                return false;
        }
        public void end(boolean interrupted) {
                this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
        }
        
}
