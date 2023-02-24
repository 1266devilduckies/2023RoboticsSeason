package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
        public TalonSRX motor = new TalonSRX(Constants.CAN.Claw.motor);
        private double clawSpeed = 0.5;
        public ClawSubsystem() {
                motor.configFactoryDefault();
                motor.setNeutralMode(NeutralMode.Coast);
                Preferences.initDouble(Constants.ClawCharacteristics.clawSpeedKey, clawSpeed);
        }
        public void loadPreferences() {
                clawSpeed = Preferences.getDouble(Constants.ClawCharacteristics.clawSpeedKey, clawSpeed);
        }
        public double getClawSpeed() {
                return clawSpeed;
        }
}
