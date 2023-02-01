package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.HomeToElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    public TalonFX leftClimber = DrivetrainSubsystem.MainLeftMotorBack;//new TalonFX(Constants.CAN.Elevator.leftClimber);
    public TalonFX rightClimber = DrivetrainSubsystem.MainRightMotorBack;//new TalonFX(Constants.CAN.Elevator.rightClimber);
    double position = 0;
    public int idxLevel = 0; //where we at in the array
    public ElevatorSubsystem() {
        leftClimber.configFactoryDefault();
        rightClimber.configFactoryDefault();
        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);
        leftClimber.config_kP(0, Constants.ElevatorCharacteristics.kP);
        rightClimber.config_kP(0, Constants.ElevatorCharacteristics.kP);
        rightClimber.setInverted(true);
        leftClimber.setInverted(false);
        setGoal(1);
        this.setDefaultCommand(new HomeToElevatorPosition(this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder position", leftClimber.getSelectedSensorPosition());
    }

    public void setGoal(double elevatorOffset) {
        position = Constants.ElevatorCharacteristics.elevatorBottomLimit + elevatorOffset * (Constants.ElevatorCharacteristics.elevatorTopLimit - Constants.ElevatorCharacteristics.elevatorBottomLimit);
        leftClimber.set(ControlMode.Position, position);
        rightClimber.set(ControlMode.Position, position);
    }
    public void setGoalAsTickOffset(double elevatorOffsetTicks) {
        position = elevatorOffsetTicks;
        leftClimber.set(ControlMode.Position, position);
        rightClimber.set(ControlMode.Position, position);
    }

    public boolean isAtTarget() {
        double err = Math.abs(leftClimber.getSelectedSensorPosition() - position);
        double errRight = Math.abs(rightClimber.getSelectedSensorPosition() - position);
        return err < 0.1 && errRight < 0.1;
    }
    public boolean isAtTarget(double setpoint) {
        double err = Math.abs(leftClimber.getSelectedSensorPosition() - setpoint);
        double errRight = Math.abs(rightClimber.getSelectedSensorPosition() - setpoint);
        return err < 0.1 && errRight < 0.1;
    }
    
    public double getLevelPercentageFromIdx(int idx) {
        double elevatorLevel = Constants.ElevatorCharacteristics.elevatorLevels[idx];
        return (elevatorLevel - Constants.ElevatorCharacteristics.elevatorBottomLimit)/(Constants.ElevatorCharacteristics.elevatorTopLimit-Constants.ElevatorCharacteristics.elevatorBottomLimit);
    }
}
