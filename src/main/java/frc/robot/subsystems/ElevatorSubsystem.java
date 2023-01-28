package frc.robot.subsystems;

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
    //CANSparkMax leftClimber = new CANSparkMax(Constants.CAN.Elevator.leftClimber, MotorType.kBrushless);
    //RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
    double currentPosition = 0; //in terms of rotations
    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();
    public ProfiledPIDController leftClimbPIDController = new ProfiledPIDController(
        Constants.ElevatorCharacteristics.kP,
        0.0,
        Constants.ElevatorCharacteristics.kD, new Constraints(500,200)); //in terms of ticks per second
    public ElevatorSubsystem() {
        //leftClimberEncoder.setPosition(0);
        DrivetrainSubsystem.MainLeftMotorBack.setSelectedSensorPosition(0);
        leftClimbPIDController.setTolerance(0.1, 0.1);
        setGoal(1);
        this.setDefaultCommand(new HomeToElevatorPosition(this));
    }

    @Override
    public void periodic() {
        //leftClimber.set(Math.sin(Timer.getFPGATimestamp()));
        SmartDashboard.putNumber("encoder position", DrivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition());
    }

    public void setGoal(double elevatorOffset) {
        double position = Constants.ElevatorCharacteristics.elevatorBottomLimit + elevatorOffset * (Constants.ElevatorCharacteristics.elevatorTopLimit - Constants.ElevatorCharacteristics.elevatorBottomLimit);
        SmartDashboard.putNumber("goal", position);
        leftClimbPIDController.setGoal(position);
    }

    public void homeToElevatorOffset() {
        DrivetrainSubsystem.MainLeftMotorBack.set(leftClimbPIDController.calculate(DrivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition()));
    }

    public boolean isAtTarget() {
        double err = Math.abs(DrivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition() - leftClimbPIDController.getGoal().position);
        SmartDashboard.putNumber("error", err);
        return err < 0.1;
    }
}
