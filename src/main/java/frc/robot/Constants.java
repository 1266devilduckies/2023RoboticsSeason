// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //All constants are in SI base units

  public static class OperatorConstants {
    public static final int port = 1;
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
      //configured for ducktails
      public static final int BL = 3;
      public static final int BR = 2;
      public static final int FL = 5;
      public static final int FR = 4;
    }
  }
  public static class DrivetrainCharacteristics {
    public static final double trackWidthMeters = Units.inchesToMeters(24.75);
    public static final double gearing = 25/3.;
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
  }
  public static class LimelightCharacteristics {
    public static final Transform2d offsetMeters = new Transform2d();
    public static final String photonVisionName = "camera";
    public static final double cameraPitchRadians = 0;
    public static final double cameraHeightMeters = 0;
  }
  public static HashMap<String, Command> eventMap = new HashMap<>();
}
