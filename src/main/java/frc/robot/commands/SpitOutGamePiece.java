package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class SpitOutGamePiece extends CommandBase {
    ClawSubsystem clawSubsystem;

    public SpitOutGamePiece(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(this.clawSubsystem);
    }
    public void initialize() {

    }
    public void execute() {
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.2);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
    }
}
