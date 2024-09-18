/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Units.*;
import frc.team3602.robot.generated.TunerConstants;



public final class Constants {
    //xbox controller port
    public final class ControllerPortConstants {
        public final static int kXboxControllerPort = 0;
    }

    //arm constants
    public final class ArmConstants {
      //motor constants
        public static final int kRightArmId = 10;
        public static final int kLeftArmId = 11;
        public static final int kMotorCurrentLimit = 30;
      //PID and Feedforward Constants
        public static double kP = 10.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static final double kS = 6.38;
        public static final double kG = 2.36;
        public static final double kV = 1.53;
        public static final double kA = 0.53;
     //conversion factor
        public static final double kHeightConvFact = (2.0 * Math.PI) / 45.0;
      //enxtended height of arms
        public static final double kExtendedHeight = 47;
      //retracted height of arms *this I added****
     public static final double kRetractedHeight = 26.5;
        
    }

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
    public static final double kP = 0.35;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 6.0;
    public static final double kG = 0.21;
    public static final double kV = 7.6;
    public static final double kA = 0.01;

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
public final class DrivetrainConstants {
    public static final double kMaxSpeed = 6.0;
    public static final double kMaxAngularRate = Math.PI;

    private final static int kPigeonId = 52;
    private final static String kCANBusName = "rio";

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kTurnGeatRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2.0;
    private static final double kSlipCurrentAmps = 100.0;
    private static final Slot0Configs kDriveGains = new Slot0Configs()
        .withKP(3.0).withKI(0.0).withKD(0.0)
        .withKS(0.0).withKV(0.0).withKA(0.0);
    private static final Slot0Configs kTurnGains = new Slot0Configs()
        .withKP(100.0).withKI(0.0).withKD(0.05)
        .withKS(0.0).withKV(1.5).withKA(0.0);
    public static final double kSpeedAt12VoltsMps = 6.0;
    private static final double kCoupleGearRatio = 3.5714285714285716;
    private static final boolean kTurnMotorInverted = true;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
        .withPigeon2Id(kPigeonId)
        .withCANbusName(kCANBusName);

    private static final SwerveModuleConstantsFactory kModuleConstants = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(kDriveGearRatio)
        .withSteerMotorGearRatio(kTurnGeatRatio)
        .withWheelRadius(kWheelRadiusInches)
        .withSlipCurrent(kSlipCurrentAmps)
        .withDriveMotorGains(kDriveGains)
        .withSteerMotorGains(kTurnGains)
        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
        .withCouplingGearRatio(kCoupleGearRatio)
        .withSteerMotorInverted(kTurnMotorInverted)
        .withFeedbackSource(SteerFeedbackType.FusedCANcoder);

    // Front left (Module 0)
    private static final int kFrontLeftTurnMotorId = 49;
    private static final int kFrontLeftDriveMotorId = 46;
    private static final int kFrontLeftEncoderId = 41;
    private static final double kFrontLeftEncoderOffset = -0.077392578125;
    private static final double kFrontLeftXPosInches = 10.25;
    private static final double kFrontLeftYPosInches = 10.25;

    public static final SwerveModuleConstants kFrontLeftModuleConstants = kModuleConstants
        .createModuleConstants(
            kFrontLeftTurnMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches),
            Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);

    // Front right (Module 1)
    private static final int kFrontRightTurnMotorId = 45;
    private static final int kFrontRightDriveMotorId = 44;
    private static final int kFrontRightEncoderId = 42;
    private static final double kFrontRightEncoderOffset = -0.343994140625;
    private static final double kFrontRightXPosInches = 10.25;
    private static final double kFrontRightYPosInches = -10.25;

    public static final SwerveModuleConstants kFrontRightModuleConstants = kModuleConstants
        .createModuleConstants(
            kFrontRightTurnMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide);

    // Back left (Module 2)
    private static final int kBackLeftTurnMotorId = 50;
    private static final int kBackLeftDriveMotorId = 51;
    private static final int kBackLeftEncoderId = 40;
    private static final double kBackLeftEncoderOffset =  -0.4482421875;
    private static final double kBackLeftXPosInches = -10.25;
    private static final double kBackLeftYPosInches = 10.25;

    public static final SwerveModuleConstants kBackLeftModuleConstants = kModuleConstants
        .createModuleConstants(
            kBackLeftTurnMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches),
            Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);

    // Back right (Module 3)
    private static final int kBackRightTurnMotorId = 47;
    private static final int kBackRightDriveMotorId = 48;
    private static final int kBackRightEncoderId = 43;
    private static final double kBackRightEncoderOffset =0.138671875;
    private static final double kBackRightXPosInches = -10.25;
    private static final double kBackRightYPosInches = -10.25;

    public static final SwerveModuleConstants kBackRightModuleConstants = kModuleConstants
        .createModuleConstants(
            kBackRightTurnMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches),
            Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);

 }
  //vision constants (used in drivetrain subsys)
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
