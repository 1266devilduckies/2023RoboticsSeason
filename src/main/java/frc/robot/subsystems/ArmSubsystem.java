package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveArm;

public class ArmSubsystem extends SubsystemBase{
        private final CANSparkMax shoulderMotor = new CANSparkMax(Constants.CAN.Arm.shoulderMotor, MotorType.kBrushless); 
        private final CANSparkMax elbowMotor = new CANSparkMax(Constants.CAN.Arm.elbowMotor, MotorType.kBrushless);
        private final SparkMaxPIDController shoulderPID = shoulderMotor.getPIDController();
        private final SparkMaxPIDController elbowPID = elbowMotor.getPIDController();
        private final RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder(); //in terms of revolutions
        private final RelativeEncoder elbowEncoder = elbowMotor.getEncoder(); //in terms of revolutions

        private double armShoulderSetpointDegrees = 0;
        private double armElbowSetpointDegrees = 0;

        public ArmSubsystem(){
                this.setDefaultCommand(new DriveArm(this));
                shoulderEncoder.setPosition(0);
                elbowEncoder.setPosition(0);
        }

        @Override
        public void periodic() {
                armShoulderSetpointDegrees = MathUtil.clamp(armShoulderSetpointDegrees, Constants.Arm.minAngleShoulder, Constants.Arm.maxAngleShoulder);
                armElbowSetpointDegrees = MathUtil.clamp(armElbowSetpointDegrees, Constants.Arm.minAngleElbow, Constants.Arm.maxAngleElbow);

                shoulderPID.setReference(armShoulderSetpointDegrees, ControlType.kPosition);
                elbowPID.setReference(armElbowSetpointDegrees, ControlType.kPosition);

                SmartDashboard.putNumber("target elbow setpoint", armElbowSetpointDegrees);
                SmartDashboard.putNumber("current elbow rotations", elbowEncoder.getPosition());

                SmartDashboard.putNumber("target shoulder setpoint", armShoulderSetpointDegrees);
                SmartDashboard.putNumber("current shoulder rotations", shoulderEncoder.getPosition());
                SmartDashboard.putBoolean("elbow at target", elbowAtTarget());
                SmartDashboard.putBoolean("shoulder at target", elbowAtTarget());
        }
        public void commandAngle(double angle) {
                armShoulderSetpointDegrees = angle;
        }

        public double getShoulderAngleSetpoint(){
                return armShoulderSetpointDegrees;
        }

        public double getElbowAngleSetpoint() {
                return armElbowSetpointDegrees;
        }

        public void ElbowCommandAngle(double angle) {
                armElbowSetpointDegrees = angle;
        }

        public double getRealAngleShoulder() {
                return shoulderEncoder.getPosition();
        }

        public double getRealAngleElbow() {
                return elbowEncoder.getPosition();
        }

        public boolean elbowAtTarget() {
                return (Math.abs(elbowEncoder.getPosition()-armElbowSetpointDegrees) < 10.);
        }

        public boolean shoulderAtTarget() {
                return (Math.abs(shoulderEncoder.getPosition()-armShoulderSetpointDegrees) < 10.);
        }
}
