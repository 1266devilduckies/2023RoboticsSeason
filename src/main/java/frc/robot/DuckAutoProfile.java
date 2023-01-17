package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DuckAutoProfile {
    private CommandBase sequence;
    private Pose2d initPose;

    public DuckAutoProfile(RobotContainer container) {
        sequence = new SequentialCommandGroup();
        initPose = new Pose2d();
    }

    public DuckAutoProfile(CommandBase commandSequence, Pose2d initialPose) {
        sequence = commandSequence;
        initPose = initialPose;
    }

    public CommandBase getAutoCommand() {
        return sequence;
    }

    public Pose2d getStartingPose() {
        return initPose;
    }
}
