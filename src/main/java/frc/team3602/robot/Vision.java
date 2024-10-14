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
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import java.io.IOException;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import static frc.team3602.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase{
  public final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private final PhotonCamera photonCamera = new PhotonCamera(kPhotonCameraName);
  private final PhotonCamera photonNote = new PhotonCamera(kNoteCameraName);
  private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, kRobotToCamera);
          SimCameraProperties props = new SimCameraProperties();

     PhotonCameraSim cameraSim = new PhotonCameraSim(photonCamera, props);

  private double lastEstimateTimestamp = 0.0;

    private final VisionSystemSim visionSim = new VisionSystemSim("main");
  private final Pose2dSupplier getSimPose;

  @FunctionalInterface
  public interface Pose2dSupplier {
    Pose2d getPose2d();
  }


  public Vision(Pose2dSupplier getSimPose) {
    this.getSimPose = getSimPose;
    configVision();
  }

  public PhotonPipelineResult getLatestResult() {
    return photonCamera.getLatestResult();
  }

  public PhotonPipelineResult getNoteResult() {
    return photonNote.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    var visionEstimate = photonPoseEstimator.update();
    double latestTimestamp = getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestamp) > 1e-5;

    if (newResult) {
      lastEstimateTimestamp = latestTimestamp;
    }

    return visionEstimate;
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
      simTargetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getTargetHeight(),
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
  }

  private void configVision() {
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
     // Setup simulated camera properties
    props.setCalibError(0.25, 0.08);
    props.setFPS(20.0);
    props.setAvgLatencyMs(35.0);
    props.setLatencyStdDevMs(5.0);

    // Setup simulated camera
    // Draw field wireframe in simulated camera view
    cameraSim.enableDrawWireframe(true);

    // Add simulated camera to vision sim
    visionSim.addCamera(cameraSim, Constants.Transforms.robotToCamera);

    // Add AprilTags to vision sim
    try {
      AprilTagFieldLayout tagLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      visionSim.addAprilTags(tagLayout);
    } catch (IOException e) {
      System.err.println(e);
    }
  }
}
