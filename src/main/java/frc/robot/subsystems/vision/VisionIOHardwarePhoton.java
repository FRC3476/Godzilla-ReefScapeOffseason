package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOHardwarePhoton implements VisionIOPhoton {

  protected final PhotonCamera camera;
  private static final double[] DEFAULT_STDDEVS =
      new double[VisionConstants.kExpectedStdDevArrayLength];

  protected final PhotonPoseEstimator estimator;
  private final String cameraName;
  private final Transform3d robotToCamera;

  public VisionIOHardwarePhoton(String cameraName, Transform3d robotToCamera) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
    camera = new PhotonCamera(cameraName);

    estimator =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagLayout,
            // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PoseStrategy.LOWEST_AMBIGUITY,
            robotToCamera);

    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public String getName() {
    return cameraName;
  }

  @Override
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOPhotonInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    List<Pose3d> aprilTagPoses = new ArrayList<>();
    List<Pose3d> poses = new ArrayList<>();
    List<PoseObservation> poseObservations = new ArrayList<>();

    for (PhotonPipelineResult result : unreadResults) {
      Optional<EstimatedRobotPose> estimatedPoseOptional = estimator.update(result);
      if (estimatedPoseOptional.isEmpty()) {
        continue;
      }
      // GET RID OF THIS LATER
      System.out.println("If you see this it is working");
      EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
      Matrix<N3, N1> stdDevs =
          AprilTagAlgorithms.getEstimationStdDevs(
              estimatedPose.estimatedPose.toPose2d(), result.getTargets());

      if (result.getMultiTagResult().isPresent()) {
        MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();

        PoseObservation observation =
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                multiTagResult.estimatedPose.ambiguity,
                multiTagResult.fiducialIDsUsed.get(0),
                stdDevs);

        for (PhotonTrackedTarget target : result.getTargets()) {
          aprilTagPoses.add(VisionConstants.fieldLayout.getTagPose(target.fiducialId).get());
        }

        poseObservations.add(observation);
        poses.add(observation.robotPose());

      } else if (!result.getTargets().isEmpty()) {
        PhotonTrackedTarget target = result.getTargets().get(0);
        aprilTagPoses.add(VisionConstants.fieldLayout.getTagPose(target.fiducialId).get());

        PoseObservation observation =
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                target.poseAmbiguity,
                target.fiducialId,
                stdDevs);

        // ADD REJECTION
        poseObservations.add(observation);
        poses.add(observation.robotPose());
      }
    }

    inputs.connected = camera.isConnected();
    inputs.aprilTagPoses = aprilTagPoses.toArray(Pose3d[]::new);
    inputs.poses = poses.toArray(Pose3d[]::new);
    inputs.poseObservations = poseObservations.toArray(PoseObservation[]::new);
  }
}
