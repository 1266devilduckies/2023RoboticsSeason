package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    CANSparkMax leftClimber = new CANSparkMax(Constants.CAN.Elevator.leftClimber, MotorType.kBrushless);
    RelativeEncoder m_leftClimberEncoder = leftClimber.getEncoder();
    public ElevatorSubsystem() {
        m_leftClimberEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        leftClimber.set(Math.sin(Timer.getFPGATimestamp()));
        SmartDashboard.putNumber("encoder position", m_leftClimberEncoder.getPosition());
    }
}
