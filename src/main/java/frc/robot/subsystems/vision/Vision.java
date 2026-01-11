package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.util.RobotTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final RobotState state;
  private final CoralPoseTracker coralPoseTracker = new CoralPoseTracker();

  public static record AprilTagCamera(
      VisionIOPhoton io,
      VisionIOPhotonInputsAutoLogged inputs,
      String cameraName,
      Transform3d robotToCamera,
      Alert disconnectedAlert) {}

  private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

  private boolean useVision = true;

  public Vision(RobotState state) {
    this.state = state;

    VisionConstants.visionSim.ifPresent(
        visionSim -> visionSim.addAprilTags(VisionConstants.kAprilTagLayout));

    Transform3d robotToCameraA =
        new Transform3d(
            new Translation3d(
                VisionConstants.kRobotToCameraAForward,
                VisionConstants.kRobotToCameraASide,
                VisionConstants.kCameraAHeightOffGroundMeters),
            new Rotation3d(
                0.0, // Roll
                -VisionConstants.kCameraAPitchRads, // Pitch
                -VisionConstants.kCameraAYawOffset.getRadians() // Yaw
                ));

    Transform3d robotToCameraB =
        new Transform3d(
            new Translation3d(
                VisionConstants.kRobotToCameraBForward,
                VisionConstants.kRobotToCameraBSide,
                VisionConstants.kCameraBHeightOffGroundMeters),
            new Rotation3d(
                0.0, // Roll
                -VisionConstants.kCameraBPitchRads, // Pitch
                -VisionConstants.kCameraBYawOffset.getRadians() // Yaw
                ));

    VisionIOSimPhoton camA = new VisionIOSimPhoton("camera1", robotToCameraA);

    aprilTagCameras.add(
        new AprilTagCamera(
            camA,
            new VisionIOPhotonInputsAutoLogged(),
            camA.getName(),
            camA.getRobotToCamera(),
            new Alert(camA.getName() + " disconnected", AlertType.kWarning)));

    VisionIOSimPhoton camB = new VisionIOSimPhoton("camera2", robotToCameraB);

    aprilTagCameras.add(
        new AprilTagCamera(
            camB,
            new VisionIOPhotonInputsAutoLogged(),
            camB.getName(),
            camB.getRobotToCamera(),
            new Alert(camB.getName() + " disconnected", AlertType.kWarning)));
  }

  public boolean isCoralDetected() {
    // return debounce.calculate(io.isCoralDetected());
    return false;
  }

  public double getCoralTx() {
    // return io.getCoralTx();
    return 0;
  }

  public double getCoralTy() {
    // return io.getCoralTy();
    return 0;
  }

  public double getCoralTxNc() {
    // return io.getCoralTxNc();
    return 0;
  }

  public double getCoralTyNc() {
    // return io.getCoralTyNc();
    return 0;
  }

  public Transform2d getCoralPositionRelativeToRobot(Pair<Double, Double> TNCs) {
    double dy =
        VisionConstants.kIntakeCameraHeight
            * Math.tan(
                Units.degreesToRadians(
                    VisionConstants.kIntakeCameraPitchDegrees + TNCs.getSecond()));

    double d0 = Math.sqrt(Math.pow(dy, 2) + Math.pow(VisionConstants.kIntakeCameraHeight, 2));

    double dx = d0 * Math.tan(Units.degreesToRadians(TNCs.getFirst()));

    return new Transform2d(dy + VisionConstants.kIntakeCameraOffset, -dx, Rotation2d.kZero);
  }

  public Pose2d calculateCoralPose(Pair<Double, Double> TNCs) {
    return RobotState.getGlobalPose().plus(getCoralPositionRelativeToRobot(TNCs));
  }

  @Override
  public void simulationPeriodic() {
    VisionConstants.visionSim.ifPresent(
        visionSim -> {
          visionSim.update(RobotState.getGlobalPose());
        });
  }

  @Override
  public void periodic() {
    double startTime = RobotTime.getTimestampSeconds();

    if (!useVision) {
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
      Logger.recordOutput("Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - startTime);
      return;
    }

    Logger.recordOutput("Vision/usingVision", true);

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTagPoses = new ArrayList<>();
    List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

    for (AprilTagCamera cam : aprilTagCameras) {
      cam.io.updateInputs(cam.inputs);
      Logger.processInputs("Vision" + "/" + cam.cameraName(), cam.inputs);

      cam.disconnectedAlert.set(!cam.inputs.connected);

      validIds.addAll(Arrays.stream(cam.inputs.aprilTagIds).boxed().toList());
      rejectedIds.addAll(Arrays.stream(cam.inputs.invalidAprilTagIds).boxed().toList());

      validPoseObservations.addAll(Arrays.asList(cam.inputs.poseObservations));
      rejectedPoseObservations.addAll(Arrays.asList(cam.inputs.invalidPoseObservations));

      validPoses.addAll(Arrays.asList(cam.inputs.poses));
      rejectedPoses.addAll(Arrays.asList(cam.inputs.invalidPoses));

      validAprilTagPoses.addAll(Arrays.asList(cam.inputs.aprilTagPoses));
      rejectedAprilTagPoses.addAll(Arrays.asList(cam.inputs.invalidAprilTagPoses));

      for (PoseObservation observation : cam.inputs.poseObservations) {
        double distanceToTag =
            VisionConstants.kAprilTagLayout
                .getTagPose(observation.id())
                .get()
                .minus(new Pose3d(RobotState.getGlobalPose()).plus(cam.robotToCamera()))
                .getTranslation()
                .getNorm();

        state.updateMegatagEstimate(
            new VisionFieldPoseEstimate(
                observation.robotPose().toPose2d(),
                observation.timestampSeconds(),
                observation.stdDevs(),
                1,
                distanceToTag));
      }
    }

    Logger.recordOutput("Vision/objectDetection/isCoralDetected", isCoralDetected());
    if (isCoralDetected()) {
      Logger.recordOutput("Vision/objectDetection/coralTx", getCoralTx());
      Logger.recordOutput("Vision/objectDetection/CoralTy", getCoralTy());
      processCoralDetections();
      CoralPoseTracker.CoralPoseObservation[] coralPoseObservations =
          coralPoseTracker.getObservations();
      Pose3d[] coralPoses = new Pose3d[coralPoseObservations.length];
      for (int i = 0; i < coralPoseObservations.length; i++) {
        coralPoses[i] = (new Pose3d(new Pose2d(coralPoseObservations[i].pose, Rotation2d.kZero)));
      }
      Logger.recordOutput("Vision/objectDetection/CoralPoses", coralPoses);
      if (coralPoseTracker.getCoralPose().isPresent()) {

        Logger.recordOutput(
            "Vision/objectDetection/bestCoralPose", coralPoseTracker.getCoralPose().get());
      }
    }

    Logger.recordOutput("Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - startTime);
  }

  public Optional<Pose2d> getCoralPose() {
    if (coralPoseTracker.getCoralPose().isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(coralPoseTracker.getCoralPose().get());
  }

  private void processCoralDetections() {
    /*if (io.getAllCoralTNCs().isPresent()) {
      ArrayList<Pair<Double, Double>> allCoralTNCs =
          io.getAllCoralTNCs().orElseGet(() -> new ArrayList<>()); // io.getAllCoralTNCs().get();
      ArrayList<Pose2d> coralPoses = new ArrayList<>();
      for (Pair<Double, Double> coralTNC : allCoralTNCs) {
        coralPoses.add(calculateCoralPose(coralTNC));
      }
      coralPoseTracker.addObservations(coralPoses);
    }*/
  }

  public void setUseVision(boolean useVision) {
    this.useVision = useVision;
  }
}
