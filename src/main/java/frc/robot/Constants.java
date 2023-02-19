// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        // All constants are in SI base units

        public static class OperatorConstants {
                public static final int port = 1;
        }

        public static class DriverConstants {
                public static final int port = 0;
                public static final double deadbandLeftJoystick = 0.05;
                public static final double deadbandRightJoystick = deadbandLeftJoystick;
                public static final int ForwardDriveAxis = 3;
                public static final int TurningDriveAxis = 2;
        }

        public static class CAN {
                public static class Drivetrain {
                        // configured for heimerdinger
                        public static final int BL = 5;
                        public static final int BR = 4;
                        public static final int FL = 3;
                        public static final int FR = 7;
                        public static final int TL = 2;
                        public static final int TR = 6;
                        // configured for ducktails
                        // public static final int BL = 2;
                        // public static final int BR = 3;
                        // public static final int FL = 4;
                        // public static final int FR = 5;
                        // public static final int TL = 20; // out of bounds
                        // public static final int TR = 60; // out of bounds
                }

                public static class Elevator {
                        public static final int leftClimber = 10;
                        public static final int rightClimber = 9;
                }

                public static class Arm {
                        public static final int armMotor = 90;
                }
                public static class Claw {
                        public static final int motor = 91;
                }
        }

        public static class Arm {
                public static final double gearing = 80.;
                public static final double armMassKg = 10.;
                public static final double armLengthMeters = 1.;
                public static final String kArmPositionKey = "arm position";
                public static final String kArmPKey = "arm kp";
                public static final String kArmGKey = "arm kg";
                public static final double minAngle = -90.0;
                public static final double maxAngle = 90.0;
        }

        public static class DrivetrainCharacteristics {
                public static final double trackWidthMeters = Units.inchesToMeters(24.75);
                public static final double gearing = 6.4;
                public static final double wheelRadiusMeters = Units.inchesToMeters(2);
                public static final double rampPGain = 0.1;
                public static final String gyroPitchPGainKey = "gyro pitch kP";
                public static final String movementPGainKey = "movement kP";
                public static double kS = 0.073213;// 0.607;
                public static double kV = 1.91111;// 1.42;
                public static double kA = 0.30399;// 0.68;
                public static double kP = 3.4369; // 0.274
                public static double maxAutoVelocityMeters = 4;
                public static double maxAutoAccelerationMeters = 3;
                public static double kD = 0.0;
                public static double speedScale = 1.0;
                public static double turnSpeedScale = 0.5;
                public static double deadband = 0.1;
        }

        public static class CameraCharacteristics {
                public static final String photonVisionName = "HD_Pro_Webcam_C920";
                public static final Transform3d robotToCamMeters = new Transform3d(
                                new Translation3d(-Units.inchesToMeters(8.5), 0.0, Units.inchesToMeters(15.5)),
                                new Rotation3d(0, Units.degreesToRadians(13), 0));
        }

        public static class ElevatorCharacteristics {
                public static final double kP = 0.25;
                public static double[] elevatorLevels = {
                                // in terms of encoder ticks
                                0,
                                25000,
                                50000
                };
                public static final double elevatorBottomLimit = 0; // in terms of ticks from reset
                public static final double elevatorTopLimit = Constants.ElevatorCharacteristics.elevatorLevels[Constants.ElevatorCharacteristics.elevatorLevels.length
                                - 1];
        }

        public static class AutoTrajectoryFileNames {
                public static final String HIGH_DOCKHIGH = "HighDockingHighPath";
                public static final String HIGH_DOCKLOW = "HighDockingLowPath";
                public static final String HIGH_FORWARD = "HighForwardPath";
                public static final String LOW_DOCKHIGH = "LowDockingHighPath";
                public static final String LOW_DOCKLOW = "LowDockingLowPath";
                public static final String LOW_FORWARD = "LowForwardPath";
                public static final String MID_FORWARD = "MidDockingForwardPath";
                public static final String MID_DOCKHIGH = "MidDockingHighPath";
                public static final String MID_DOCKLOW = "MidDockingLowPath";

        }

        public static class ClawCharacteristics {
                public static int forwardChannel = 0;
                public static int reverseChannel = 1;
        }

        public static HashMap<String, Command> eventMap = new HashMap<>();
}
