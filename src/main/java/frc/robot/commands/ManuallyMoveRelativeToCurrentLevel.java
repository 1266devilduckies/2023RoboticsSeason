package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManuallyMoveRelativeToCurrentLevel extends CommandBase {
    private ElevatorSubsystem elevatorSubsystem;
    private double goal;
    private final double kSpeed = 0.01;
    private double lowerLimit;
    private double upperLimit;
    public ManuallyMoveRelativeToCurrentLevel(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(this.elevatorSubsystem);
    }

    public void initialize() {
        goal = this.elevatorSubsystem.getLevelPercentageFromIdx(this.elevatorSubsystem.idxLevel);
        lowerLimit = 0.0;
        upperLimit = 1.0;
        if (this.elevatorSubsystem.idxLevel == 0) {
            double a = this.elevatorSubsystem.getLevelPercentageFromIdx(0);
            double b = this.elevatorSubsystem.getLevelPercentageFromIdx(1);
            upperLimit = a+0.5*(b-a);
        } else if (this.elevatorSubsystem.idxLevel == Constants.ElevatorCharacteristics.elevatorLevels.length-1) {
            double a = this.elevatorSubsystem.getLevelPercentageFromIdx(this.elevatorSubsystem.idxLevel - 1);
            double b = this.elevatorSubsystem.getLevelPercentageFromIdx(this.elevatorSubsystem.idxLevel);
            lowerLimit = b-0.5*(b-a);
        } else {
            double a = this.elevatorSubsystem.getLevelPercentageFromIdx(this.elevatorSubsystem.idxLevel-1);
            double b = this.elevatorSubsystem.getLevelPercentageFromIdx(this.elevatorSubsystem.idxLevel+1);

            double current = this.elevatorSubsystem.getLevelPercentageFromIdx(this.elevatorSubsystem.idxLevel);
            lowerLimit = a+0.5*(current-a);
            upperLimit = b-0.5*(b-current);
        }
    }

    public void execute() {
        double position = -RobotContainer.operatorJoystick.getRawAxis(1);
        if (position > 0.25 && goal < upperLimit) {
            goal = goal + kSpeed;
        } else if (position < -0.25 && goal > lowerLimit) {
            goal = goal - kSpeed;
        }
        double mappedGoal = MathUtil.clamp(goal + this.elevatorSubsystem.getLevelPercentageFromIdx(this.elevatorSubsystem.idxLevel), lowerLimit, upperLimit);
        SmartDashboard.putNumber("goal", goal);
        SmartDashboard.putNumber("lower limit", lowerLimit);
        SmartDashboard.putNumber("upper limit", upperLimit);
        elevatorSubsystem.setGoal(mappedGoal);
    }
}
