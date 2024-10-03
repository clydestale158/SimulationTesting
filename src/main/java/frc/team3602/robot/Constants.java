/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.team3602.robot.generated.TunerConstants;



public final class Constants {

  public class Transforms {
    public static Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(0, 0, 0.5), // Centered on the robot, 0.5m up
            new Rotation3d(0, Math.toRadians(-15), 0)); // Pitched 15 deg up
  }

   public class Drivetrain {
    public static double kMaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // m/s
    public static double kMaxAngularRate = 1.5 * Math.PI; // rad/s
    public static double kDeadband = 0.1;
  }

}
