// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

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
                public static final int movementAxis = 5;
                public static final double movementDeadband = 0.5;
                public static final int elbowMovementAxis = 1;
        }

        public static class DriverConstants {
                public static final int port = 0;
                public static final double deadbandLeftJoystick = 0.05;
                public static final double deadbandRightJoystick = deadbandLeftJoystick;
                public static final int ForwardDriveAxis = 1;
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
                }

                public static class Arm {
                        public static final int elbowMotor = 10;
                        public static final int shoulderMotor = 8;
                }
                public static class Claw {
                        public static final int motor = 9;
                }
        }

        public static class Arm {
                public static final double minAngleShoulder = 0.0;
                public static final double maxAngleShoulder = 180.0;
                public static final double maxAngleElbow = 165.0;
                public static final double minAngleElbow = 0.0;
        }

        public static class DrivetrainCharacteristics {
                public static final double trackWidthMeters = Units.inchesToMeters(18);
                public static final double gearing = 6.4;
                public static final double wheelRadiusMeters = Units.inchesToMeters(2);
                public static final double rampPGain = 0.1;
                public static final double kSAngular = -0.1;
                public static final double kS = 0.17825;
                public static final double kV = 2.2146;
                public static final double kA = 0.278;
                public static final double kP = 0.37182;
                //changed speed from 2.5 to 1.5 cause moad told me to
                public static final double maxAutoVelocityMeters = 1.5;
                public static final double maxAutoAccelerationMeters = 1.5;
                public static final double speedScale = 0.8;
                public static final double deadband = 0.1;
        }

        public static class AutoTrajectoryFileNames {
                public static final String HIGH_TAXI = "HighTaxiPath";
                public static final String MID_TAXI = "MidTaxiPath";
                public static final String LOW_TAXI = "LowTaxiPath";
                public static final String MID_BALANCE = "MidBalance";
                public static final String CONE_TAXI = "ConeTaxi";
                public static final String DOCK = "DockPath";
        }

        public static class ClawCharacteristics {
                public static final double clawTime = 0.5;
                public static final double clawSpeed = 0.8;
                public static final double gamePieceSpitOutTime = 0.5;
        }


        public static HashMap<String, Command> eventMap = new HashMap<>();
}
