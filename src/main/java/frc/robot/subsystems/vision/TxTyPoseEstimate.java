package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;

/**
 * Represents a local pose estimate calculated using trig-based TxTy approach.
 *
 * <p>This class calculates a robot pose from a single AprilTag using:
 *
 * <ul>
 *   <li>Unambiguous 3D distance from SolvePNP
 *   <li>Horizontal/vertical angles (tx/ty) from tag position in frame
 *   <li>Robot gyro rotation at observation time
 *   <li>Known camera mounting position and orientation
 * </ul>
 */
public class TxTyPoseEstimate {
  private final Pose2d localPose;
  private final double distance2d;
  private final int tagId;
  private final double timestampSeconds;

  /**
   * Creates a TxTy pose estimate.
   *
   * @param localPose The calculated robot pose on the field
   * @param distance2d The 2D (horizontal) distance from robot to tag
   * @param tagId The AprilTag ID used for this estimate
   * @param timestampSeconds When this observation was captured
   */
  public TxTyPoseEstimate(
      Pose2d localPose, double distance2d, int tagId, double timestampSeconds) {
    this.localPose = localPose;
    this.distance2d = distance2d;
    this.tagId = tagId;
    this.timestampSeconds = timestampSeconds;
  }

  /**
   * Creates a TxTy pose estimate from an observation.
   *
   * <p>Algorithm:
   *
   * <ol>
   *   <li>Calculate 2D distance using: distance2d = distance3d * cos(cameraPitch + ty)
   *   <li>Get tag pose from field layout
   *   <li>Calculate camera-to-tag rotation using robot yaw, camera yaw offset, and tx
   *   <li>Calculate field-to-camera translation by projecting backwards from tag
   *   <li>Transform from camera pose to robot pose using camera mounting transform
   * </ol>
   *
   * @param observation The TxTy observation
   * @param robotPoseAtTime The robot's odometry pose at observation time (for gyro angle)
   * @param robotToCamera The transform from robot center to camera
   * @param cameraPitchRadians The camera's mounting pitch angle
   * @return A TxTyPoseEstimate if successful, or empty if tag not found in field layout
   */
  public static Optional<TxTyPoseEstimate> fromObservation(
      TxTyObservation observation,
      Pose2d robotPoseAtTime,
      Transform2d robotToCamera,
      double cameraPitchRadians) {

    // Get tag pose from field layout
    Optional<Pose3d> maybeTagPose3d =
        VisionConstants.kAprilTagLayout.getTagPose(observation.tagId());
    if (maybeTagPose3d.isEmpty()) {
      return Optional.empty();
    }
    Pose2d tagPose = maybeTagPose3d.get().toPose2d();

    // Calculate 2D (horizontal) distance from 3D distance
    // distance2d = distance3d * cos(camera_pitch + ty)
    double distance2d = observation.distance3d() * Math.cos(cameraPitchRadians + observation.ty());

    // Calculate camera-to-tag rotation
    // This is: robot rotation + camera yaw offset + horizontal angle to tag
    Rotation2d cameraToTagRotation =
        robotPoseAtTime
            .getRotation()
            .plus(robotToCamera.getRotation())
            .plus(Rotation2d.fromRadians(observation.tx()));

    // Calculate field-to-camera translation
    // Start at tag, go backwards (opposite direction) by distance2d
    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose.getTranslation(), cameraToTagRotation.plus(Rotation2d.kPi))
            .transformBy(new Transform2d(new Translation2d(distance2d, 0.0), Rotation2d.kZero))
            .getTranslation();

    // Create camera pose on field (using robot's gyro rotation, not camera rotation)
    Pose2d cameraPose =
        new Pose2d(fieldToCameraTranslation, robotPoseAtTime.getRotation().plus(robotToCamera.getRotation()));

    // Transform from camera pose to robot pose
    // Apply inverse of robot-to-camera transform
    Pose2d robotPose =
        cameraPose.transformBy(
            new Transform2d(robotToCamera.getTranslation().unaryMinus(), robotToCamera.getRotation().unaryMinus()));

    // Use gyro angle at observation time for robot rotation (not calculated rotation)
    robotPose = new Pose2d(robotPose.getTranslation(), robotPoseAtTime.getRotation());

    return Optional.of(
        new TxTyPoseEstimate(robotPose, distance2d, observation.tagId(), observation.timestampSeconds()));
  }

  public Pose2d getLocalPose() {
    return localPose;
  }

  public double getDistance2d() {
    return distance2d;
  }

  public int getTagId() {
    return tagId;
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }
}

