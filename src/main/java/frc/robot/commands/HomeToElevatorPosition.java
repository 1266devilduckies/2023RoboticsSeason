package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeToElevatorPosition extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;
    double goal = 0;
    double percentPerSecond = 2;
    public HomeToElevatorPosition(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.homeToElevatorOffset();
        double position = -RobotContainer.operatorJoystick.getRawAxis(1);
        if (position > 0.5) {
            goal = goal + percentPerSecond * 0.02;
        } else if (position < -0.5) {
            goal = goal - percentPerSecond * 0.02;
        }
        goal = MathUtil.clamp(goal, 0, 1);
        elevatorSubsystem.setGoal(goal);
        // if (elevatorSubsystem.isAtTarget()) {
        //     if (goal == 1) {
        //         goal = 0;
        //     } else {
        //         goal = 1;
        //     }
        //     double position = -RobotContainer.operatorJoystick.getRawAxis(1);
        //     elevatorSubsystem.setGoal(goal);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
