/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;



import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.team3602.robot.generated.TunerConstants;



public final class Constants {
    //xbox controller port
    public final class ControllerPortConstants {
        public final static int kXboxControllerPort = 0;
    }

    // //arm constants
    // public final class ArmConstants {
    //   //motor constants
    //     public static final int kRightArmId = 10;
    //     public static final int kLeftArmId = 11;
    //     public static final int kMotorCurrentLimit = 30;
    //   //PID and Feedforward Constants
    //     public static double kP = 10.0;
    //     public static double kI = 0.0;
    //     public static double kD = 0.0;

    //     public static final double kS = 6.38;
    //     public static final double kG = 2.36;
    //     public static final double kV = 1.53;
    //     public static final double kA = 0.53;
    //  //conversion factor
    //     public static final double kHeightConvFact = (2.0 * Math.PI) / 45.0;
    //   //enxtended height of arms
    //     public static final double kExtendedHeight = 47;
    //   //retracted height of arms *this I added****
    //  public static final double kRetractedHeight = 26.5;
        
    // }

    //intake constants
    public final class IntakeConstants {
      //motor constants
        public static final int kIntakeMotorId = 5;
        public static final int kIntakeMotorCurrentLimit = 30;
      //sensor constants
        public static final int kBeamSensorId = 3;
        public static final int kColorSensorId = 1;
    }

    //shooter constants
  public final class shooterConstants{
    //motor constants
    public static final int kTopShooterMotorId = 2;
    public static final int kBottomShooterMotorId = 3;

    public static final int kShooterCurrentLimit = 25;

    //conversion factor
    public static final double kTopConvFactor = (Math.PI * 4.0);

    //PID and feedforward constants
    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.1;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  //pivot constants
  public final class pivotConstants {
    //motor constants
    public static final int kPivotLeaderId = 4;
    public static final int kPivotFollowerId = 6;

    public static final int kPivotCurrentLimit = 15;

    public static double kPivotConvFactor = 360;
    public static double kAbsoluteOffset = 0;

    //PID and feedforward constants
    public static final double kP = 0.6; 
    public static final double kI = 0.0;
    public static final double kD = 0.04;

    public static final double kS = 5.0;
    public static final double kG = 3; 
    public static final double kV = 0.88; 
    public static final double kA = 0.11; 

    //SIM PID and feedforward constants
    public static final double simKP = 0.9;
    public static final double simKI = 0.0;
    public static final double simKD = 0.04;

    public static final double simKS = 5.0;
    public static final double simKG = 19;
    public static final double simKV = 0.88;
    public static final double simKA = 0.11;

    //avoiding pid constants
    public static final double kHighVolts = 7;
    public static final double kLowVolts = 3;

    //other constants
        //idk why these two wont work :(
    //public static final Measure<Angle> kInFramePos = Degrees.of(45);
    //public static final Measure<Angle> kPickupPos = Degrees.of(90);

    public static final double kMaxVelocity = 3; // meters/second 0.15
    public static final double kMaxAcceleration = 6; //meters/second^2 0.5
  }

  //vision constants (used in old drivetrain subsys)
  public final class VisionConstants {
    public static final String kPhotonCameraName = "photonvision";
    public static final String kNoteCameraName = "photon_note";

    // Camera mounted facing forward, half a meter forward of center, half a meter
    // up from center. TODO: Measure this
    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0.0, 0.0, 0.0));

    public static final Measure<Distance> kCameraHeight = Inches.of(10.75);
    public static final Measure<Angle> kCameraPitch = Degrees.of(20); //23.5

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final String PHOTON_CAMERA_NAME = "photonvision";

    public static final double CAMERA_HEIGHT_METERS = Units.feetToMeters(1.4375);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(46.0);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.0);

    public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  }

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
