package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ClawHoldPosition;

public class ClawSubsystem extends SubsystemBase {
        public TalonFX motor = new TalonFX(Constants.CAN.Claw.motor);
        public boolean suckingInCube = false;
        
        public ClawSubsystem() {
                motor.configFactoryDefault();
                motor.setNeutralMode(NeutralMode.Brake);
                motor.setInverted(true);
                motor.config_kP(0, 0.02);
                setDefaultCommand(new ClawHoldPosition(this));
        }
        
        public double getClawSpeed() {
                return Constants.ClawCharacteristics.clawSpeed;
        }
        public double getClawTime(){
                return Constants.ClawCharacteristics.clawTime;
        }
}