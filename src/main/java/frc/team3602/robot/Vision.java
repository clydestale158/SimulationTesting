/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import static frc.team3602.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase{

  private final Pose2dSupplier getSimPose;
 public final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
   private final PhotonCamera tagCamera = new PhotonCamera(kPhotonCameraName);
 private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, tagCamera, kRobotToCamera);
  private double lastEstimateTimestamp = 0.0;

  private final VisionSystemSim visionSim = new VisionSystemSim("main");
  ////private TargetModel targetModel = TargetModel.kAprilTag36h11;
  ////private Pose3d exampleTargetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  ////VisionTargetSim exampleVisionTarget = new VisionTargetSim(exampleTargetPose, targetModel);
  
  private SimCameraProperties simProps = new SimCameraProperties();
  private PhotonCameraSim cameraSim = new PhotonCameraSim(tagCamera, simProps);




  @FunctionalInterface
  public interface Pose2dSupplier {
    Pose2d getPose2d();
  }

  public Vision(Pose2dSupplier getSimPose) {
    this.getSimPose = getSimPose;
    visionSim.addAprilTags(kFieldLayout);
    visionSim.addCamera(cameraSim, kRobotToCamera);
    ////visionSim.addVisionTargets(exampleVisionTarget);

    simProps.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    simProps.setCalibError(0.25, 0.08);
    simProps.setFPS(20);
    simProps.setAvgLatencyMs(35.0);
    simProps.setLatencyStdDevMs(5);


    //Stream is at localhost:1181, Processed stream is at localhost:1182, OR in the CameraServer tab of shuffleboard
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);




    // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    //  // Setup simulated camera properties
    // props.setCalibError(0.25, 0.08);
    // props.setFPS(20.0);
    // props.setAvgLatencyMs(35.0);
    // props.setLatencyStdDevMs(5.0);

    // // Setup simulated camera
    // // Draw field wireframe in simulated camera view
    // cameraSim.enableDrawWireframe(true);

    // // Add simulated camera to vision sim
    // visionSim.addCamera(cameraSim, Constants.Transforms.robotToCamera);

    // // Add AprilTags to vision sim
    // try {
    //   AprilTagFieldLayout tagLayout =
    //       AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    //   visionSim.addAprilTags(tagLayout);
    // } catch (IOException e) {
    //   System.err.println(e);
    // }
  }

  public PhotonPipelineResult getLatestResult() {
    return tagCamera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d prevEstimatedRobotPose) {
    // var visionEstimate = photonPoseEstimator.update();
    // double latestTimestamp = getLatestResult().getTimestampSeconds();
    // boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestamp) > 1e-5;

    // if (newResult) {
    //   lastEstimateTimestamp = latestTimestamp;
    // }
    //     return visionEstimate;


    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();
  }



  public double getTargetHeight() {
    double targetHeight;
    
    var result = getLatestResult();

    if (result.hasTargets()) {
      targetHeight = kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ();
    } else {
      targetHeight = 0.0;
    }

    return targetHeight;
  }

  public double getTargetDistance() {
    double targetDistance;

    var result = getLatestResult();

    if (result.hasTargets()) {
      targetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getTargetHeight(),
          kCameraPitch.in(Radians), Units.degreesToRadians(result.getBestTarget().getPitch()));
    } else {
      targetDistance = 0.0;
    }

    return targetDistance;
  }


  public double getSimTargetHeight() {
    double simTargetHeight;

    var simResult = cameraSim.getCamera().getLatestResult();
        if (simResult.hasTargets()) {
      simTargetHeight = kFieldLayout.getTagPose(simResult.getBestTarget().getFiducialId()).get().getZ();
    } else {
      simTargetHeight = 0.0;
    }
    return simTargetHeight;
  }

    public double simGetTargetDistance() {
    double simTargetDistance;

    var simResult = cameraSim.getCamera().getLatestResult();

    if (simResult.hasTargets()) {
      simTargetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getSimTargetHeight(),
          kCameraPitch.in(Radians), Units.degreesToRadians(simResult.getBestTarget().getPitch()));
    } else {
      simTargetDistance = 0.0;
    }

    return simTargetDistance;
  }
  
  @Override
  public void simulationPeriodic() {
    // Update the vision system with the simulated robot pose
   visionSim.update(getSimPose.getPose2d());
   visionSim.getDebugField();
  }
}
