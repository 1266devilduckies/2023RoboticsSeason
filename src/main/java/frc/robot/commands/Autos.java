// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  //format of sequential command group embedding the profile, starting pose
  public static Object[] templateAuto() {
    Object[] data = {Commands.sequence(),
                    new Pose2d()};
    return data;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
