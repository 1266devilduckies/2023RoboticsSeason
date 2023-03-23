package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class GrabGamePiece extends CommandBase {
    ClawSubsystem clawSubsystem;
    double startTime;
    boolean cubeState;
    public GrabGamePiece(ClawSubsystem clawSubsystem, boolean isCube) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(this.clawSubsystem);
        System.out.println(this.clawSubsystem.suckingInCube);
        cubeState = isCube;
    }
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        this.clawSubsystem.suckingInCube = !cubeState;
    }
    public void execute() {
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, this.clawSubsystem.getClawSpeed() * (this.clawSubsystem.suckingInCube ? -1 : 1));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.clawSubsystem.motor.getStatorCurrent()) > 60.0 && ((Timer.getFPGATimestamp() - startTime) > 0.25);
    }
    public void end(boolean interrupted) {
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
    }
}
