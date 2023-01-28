package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeToElevatorPosition extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;
    double goal = 1;
    public HomeToElevatorPosition(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.homeToElevatorOffset();
        if (elevatorSubsystem.isAtTarget()) {
            if (goal == 1) {
                goal = 0;
            } else {
                goal = 1;
            }
            elevatorSubsystem.setGoal(goal);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
