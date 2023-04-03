package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class DriveArm extends CommandBase {
        ArmSubsystem armSubsystem;
        double deadband;

        public DriveArm(ArmSubsystem armSubsystem) {
                this.armSubsystem = armSubsystem;
                this.deadband = Constants.OperatorConstants.movementDeadband;
                addRequirements(this.armSubsystem);
        }
        public void execute() {
                // double speed = 45.0; //degrees per second
                // double movementAxis = -RobotContainer.operatorJoystick.getRawAxis(Constants.OperatorConstants.movementAxis);
                // double elbowMovementAxis = -RobotContainer.operatorJoystick.getRawAxis(Constants.OperatorConstants.elbowMovementAxis);
                
                // if (movementAxis > this.deadband) {
                //         this.armSubsystem.commandAngle(this.armSubsystem.getShoulderAngleSetpoint() + speed * 0.02);
                // } else if (movementAxis < -this.deadband) {
                //         this.armSubsystem.commandAngle(this.armSubsystem.getShoulderAngleSetpoint() - speed * 0.02);
                // }

                // if (elbowMovementAxis > this.deadband) {
                //         this.armSubsystem.ElbowCommandAngle(this.armSubsystem.getElbowAngleSetpoint() + speed * 0.02);
                // } else if (elbowMovementAxis < -this.deadband) {
                //         this.armSubsystem.ElbowCommandAngle(this.armSubsystem.getElbowAngleSetpoint() - speed * 0.02);
                // }
        }

        public boolean isFinished() {
                return false;
        }
}
