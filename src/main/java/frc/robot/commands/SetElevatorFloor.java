package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorFloor extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;
    private double tickLevel;
    public SetElevatorFloor(ElevatorSubsystem subsystem, int idx) {
        this.elevatorSubsystem = subsystem;
        tickLevel = Constants.elevatorLevels[idx];
        addRequirements(elevatorSubsystem);
    }

    public void execute() {
        this.elevatorSubsystem.leftClimber.set(ControlMode.Position, tickLevel);
        this.elevatorSubsystem.rightClimber.set(ControlMode.Position, tickLevel);
    }

    public boolean isFinished() {
        return this.elevatorSubsystem.isAtTarget(tickLevel);
    }
}
