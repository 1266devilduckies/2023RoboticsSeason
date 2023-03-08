package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
        private PhotonCamera photonCamera;
        private PhotonPoseEstimator photonPoseEstimator;
        private AprilTagFieldLayout aprilTagFieldLayout;

        private DrivetrainSubsystem drivetrainSubsystem;

        public VisionSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
                this.drivetrainSubsystem = drivetrainSubsystem;
                this.photonCamera = new PhotonCamera("camera");
                try {
                        aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                } catch (IOException e) {
                        e.printStackTrace();
                }

                aprilTagFieldLayout.setOrigin(DriverStation.getAlliance() == Alliance.Red
                                ? OriginPosition.kRedAllianceWallRightSide
                                : OriginPosition.kBlueAllianceWallRightSide);

                photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera,
                                new Transform3d(
                                                new Translation3d(-Units.inchesToMeters(8.5), 0.0,
                                                                Units.inchesToMeters(15.5)),
                                                new Rotation3d(0, Units.degreesToRadians(12), 0)));
        }

        @Override
        public void periodic() {
                this.drivetrainSubsystem.getPose();
                Pose2d previousPose = this.drivetrainSubsystem.getPose();
                if (previousPose != null) {
                        photonPoseEstimator.setReferencePose(previousPose);
                        double currentTime = Timer.getFPGATimestamp();
                        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
                        if (result.isPresent()) {
                                Pose2d estimatedPoseMeters = result.get().estimatedPose.toPose2d();
                                SmartDashboard.putString("estimated pose meters", estimatedPoseMeters.toString());
                                this.drivetrainSubsystem.addApriltagMeasurement(estimatedPoseMeters, currentTime);
                        }
                }
        }
}
