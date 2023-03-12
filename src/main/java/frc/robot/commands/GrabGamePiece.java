package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class GrabGamePiece extends CommandBase {
    ClawSubsystem clawSubsystem;
    double startTime;

    public GrabGamePiece(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(this.clawSubsystem);
    }
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }
    public void execute() {
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, -0.2);
        SmartDashboard.putNumber("Stator", this.clawSubsystem.motor.getStatorCurrent());
    }
    @Override
    public boolean isFinished() {
        return this.clawSubsystem.motor.getStatorCurrent() > 3.0 && (Timer.getFPGATimestamp() - startTime) > 0.25;
    }
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("grab", false);
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
    }
}
