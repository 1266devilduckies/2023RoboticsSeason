package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
        private VictorSPX motor = new VictorSPX(Constants.CAN.Claw.motor);
        private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClawCharacteristics.forwardChannel, Constants.ClawCharacteristics.reverseChannel);
        public ClawSubsystem() {
                motor.configFactoryDefault();
                motor.setNeutralMode(NeutralMode.Coast);
        }
        
        public void toggleSolenoid() {
                if (solenoid.get() == Value.kForward) {
                        solenoid.set(Value.kReverse);
                } else {
                        solenoid.set(Value.kForward);
                }
        }
        public void erectSolenoid() {
                solenoid.set(Value.kForward);
        }
        public void retractSolenoid() {
                solenoid.set(Value.kReverse);
        }
        public void spinMotor() {
                motor.set(ControlMode.PercentOutput, 1);
        }
        public void stopMotor() {
                motor.set(ControlMode.PercentOutput, 0);
        }
}
