package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeToElevatorPosition extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;
    double goal = 0;
    double percentPerSecond = 50;
    public HomeToElevatorPosition(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        double position = -RobotContainer.operatorJoystick.getRawAxis(1);
        if (position > 0.5) {
            goal = goal + percentPerSecond * 0.02;
        } else if (position < -0.5) {
            goal = goal - percentPerSecond * 0.02;
        }

        // if (elevatorSubsystem.idxLevel == 0) {
        //     double upperLevelPercentage = elevatorSubsystem.getLevelPercentageFromIdx(
        //         elevatorSubsystem.idxLevel+1
        //     );
        //     double currentLevelPercentage = elevatorSubsystem.getLevelPercentageFromIdx(elevatorSubsystem.idxLevel);
        //     goal = MathUtil.clamp(goal, currentLevelPercentage, upperLevelPercentage - ((currentLevelPercentage-upperLevelPercentage)/2.0));
        // } else if (elevatorSubsystem.idxLevel == Constants.elevatorLevels.length) {
        //     double bottomLevelPercentage = elevatorSubsystem.getLevelPercentageFromIdx(
        //         elevatorSubsystem.idxLevel-1
        //     );
        //     double currentLevelPercentage = elevatorSubsystem.getLevelPercentageFromIdx(elevatorSubsystem.idxLevel);
        //     goal = MathUtil.clamp(goal, bottomLevelPercentage + ((currentLevelPercentage-bottomLevelPercentage)/2.0)
        //     , currentLevelPercentage);
        // } else {
        //     double upperLevelPercentage = elevatorSubsystem.getLevelPercentageFromIdx(
        //         elevatorSubsystem.idxLevel+1
        //     );
        //     double bottomLevelPercentage = elevatorSubsystem.getLevelPercentageFromIdx(
        //         elevatorSubsystem.idxLevel-1
        //     );
        //     double currentLevelPercentage = elevatorSubsystem.getLevelPercentageFromIdx(elevatorSubsystem.idxLevel);
        //     goal = MathUtil.clamp(goal, bottomLevelPercentage + ((currentLevelPercentage-bottomLevelPercentage)/2.0),
        //     upperLevelPercentage - ((currentLevelPercentage-upperLevelPercentage)/2.0));
        // }
        elevatorSubsystem.setGoal(goal);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
