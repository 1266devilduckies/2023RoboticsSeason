package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeToElevatorPosition extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;

    public HomeToElevatorPosition(ElevatorSubsystem subsystem) {
        this.elevatorSubsystem = subsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {

        if(!RobotContainer.operatorJoystick.leftBumper().getAsBoolean()){
            elevatorSubsystem.setGoalAsTickOffset(Constants.ElevatorCharacteristics.elevatorLevels[this.elevatorSubsystem.idxLevel]);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
