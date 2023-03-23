package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class SpitOutGamePiece extends CommandBase {
        ClawSubsystem clawSubsystem;
        double startTime;
        boolean suckInOverride = false;
        boolean isCubeType = false; // ignored unless overrided

        public SpitOutGamePiece(ClawSubsystem clawSubsystem) {
                this.clawSubsystem = clawSubsystem;
                addRequirements(this.clawSubsystem);
        }

        public SpitOutGamePiece(ClawSubsystem clawSubsystem, boolean isCube) {
                suckInOverride = true;
                isCubeType = isCube;
        }

        public void initialize() {
                SmartDashboard.putNumber("started", startTime);
                startTime = Timer.getFPGATimestamp();
        }

        public void execute() {
                if (suckInOverride == false) {
                        this.clawSubsystem.motor.set(ControlMode.PercentOutput,
                                        Constants.ClawCharacteristics.gamePieceSpitOutTime
                                                        * (this.clawSubsystem.suckingInCube ? 1 : -1));
                } else {
                        this.clawSubsystem.motor.set(ControlMode.PercentOutput,
                                        Constants.ClawCharacteristics.gamePieceSpitOutTime * (isCubeType ? 1 : -1));
                }
        }

        @Override
        public boolean isFinished() {
                return false;
        }

        public void end(boolean interrupted) {
                SmartDashboard.putNumber("ended", Timer.getFPGATimestamp());
                this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
        }
}
