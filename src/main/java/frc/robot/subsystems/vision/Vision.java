package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.util.RobotTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIOLimelight io;
  private final VisionIOPhoton pio;
  private final RobotState state;
  private final VisionIOLimelight.VisionIOInputs inputs = new VisionIOLimelight.VisionIOInputs();
  private final Debouncer debounce = new Debouncer(0.25);
  private final CoralPoseTracker coralPoseTracker = new CoralPoseTracker();

  public static record AprilTagCamera(
      VisionIOPhoton io,
      VisionIOPhotonInputsAutoLogged inputs,
      String cameraName,
      Transform3d robotToCamera,
      Alert disconnectedAlert) {}

  private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

  private boolean useVision = true;

  public Vision(VisionIOLimelight io, RobotState state) {
    this.io = io;
    this.pio = null;
    this.state = state;
  }

  // new!! below - - - - - - - - - - - - - - - -

  public Vision(RobotState state) {
    // this.io = io;
    this.io = null;
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
                VisionConstants.kCameraAYawOffset.getRadians() // Yaw
                ));

    this.pio = new VisionIOSimPhoton("camera1", robotToCameraA);

    Alert disconnectedAlert = new Alert(pio.getName() + " disconnected", AlertType.kWarning);

    aprilTagCameras.add(
        new AprilTagCamera(
            pio,
            new VisionIOPhotonInputsAutoLogged(),
            pio.getName(),
            pio.getRobotToCamera(),
            disconnectedAlert));
  }

  public boolean isCoralDetected() {
    // return debounce.calculate(io.isCoralDetected());
    return false;
  }

  public double getCoralTx() {
    return io.getCoralTx();
  }

  public double getCoralTy() {
    return io.getCoralTy();
  }

  public double getCoralTxNc() {
    return io.getCoralTxNc();
  }

  public double getCoralTyNc() {
    return io.getCoralTyNc();
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

  /** Fuses two vision pose estimates using inverse-variance weighting. */
  private VisionFieldPoseEstimate fuseEstimates(
      VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {
    // Ensure b is the newer measurement
    if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
      VisionFieldPoseEstimate tmp = a;
      a = b;
      b = tmp;
    }

    // Preview both estimates to the same timestamp
    Transform2d a_T_b =
        state
            .getFieldToRobot(b.getTimestampSeconds())
            .get()
            .minus(state.getFieldToRobot(a.getTimestampSeconds()).get());

    Pose2d poseA = a.getVisionRobotPoseMeters().transformBy(a_T_b);
    Pose2d poseB = b.getVisionRobotPoseMeters();

    // Inverse‑variance weighting
    var varianceA = a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
    var varianceB = b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

    Rotation2d fusedHeading = poseB.getRotation();
    if (varianceA.get(2, 0) < VisionConstants.kLargeVariance
        && varianceB.get(2, 0) < VisionConstants.kLargeVariance) {
      fusedHeading =
          new Rotation2d(
              poseA.getRotation().getCos() / varianceA.get(2, 0)
                  + poseB.getRotation().getCos() / varianceB.get(2, 0),
              poseA.getRotation().getSin() / varianceA.get(2, 0)
                  + poseB.getRotation().getSin() / varianceB.get(2, 0));
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getTranslation().getX() * weightAx
                        + poseB.getTranslation().getX() * weightBx)
                    / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy
                        + poseB.getTranslation().getY() * weightBy)
                    / (weightAy + weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = a.getNumTags() + b.getNumTags();
    double distance = (a.getDistanceToTag() + b.getDistanceToTag()) / 2;
    double time = b.getTimestampSeconds();

    return new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags, distance);
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
    // io.updateInputs(inputs);
    // pio.updateInputs(inputs);

    // logCameraInputs("Vision/CameraA", inputs.cameraA);
    // logCameraInputs("Vision/CameraB", inputs.cameraB);

    // var maybeMTA = processCamera(inputs.cameraA, "CameraA", VisionConstants.kRobotToCameraA);
    // var maybeMTB = processCamera(inputs.cameraB, "CameraB", VisionConstants.kRobotToCameraB);

    if (!useVision) {
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
      Logger.recordOutput("Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - startTime);
      return;
    }

    Logger.recordOutput("Vision/usingVision", true);

    // --------------------------------------------------------------------------------------------------------------------------

    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

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

      // validCorners.addAll(Arrays.asList(cam.inputs.validCorners));
      // rejectedCorners.addAll(Arrays.asList(cam.inputs.rejectedCorners));

      // validIds.addAll(Arrays.stream(cam.inputs.validIds).boxed().toList());
      // rejectedIds.addAll(Arrays.stream(cam.inputs.rejectedIds).boxed().toList());

      validPoseObservations.addAll(Arrays.asList(cam.inputs.poseObservations));
      rejectedPoseObservations.addAll(Arrays.asList(cam.inputs.invalidPoseObservations));

      validPoses.addAll(Arrays.asList(cam.inputs.poses));
      rejectedPoses.addAll(Arrays.asList(cam.inputs.invalidPoses));

      validAprilTagPoses.addAll(Arrays.asList(cam.inputs.aprilTagPoses));
      // rejectedAprilTagPoses.addAll(Arrays.asList(cam.inputs.rejectedAprilTagPoses));

      for (PoseObservation observation : cam.inputs.poseObservations) {

        // if ((observation.method() == PoseEstimationMethod.TRIG)
        //    || (observation.method() == PoseEstimationMethod.CONSTRAINED)) {
        //  BobotState.offerConstrainedVisionObservation(observation);
        // } else {
        // BobotState.offerGlobalVisionObservation(observation);
        // }
        state.updateMegatagEstimate(
            new VisionFieldPoseEstimate(
                observation.robotPose().toPose2d(),
                observation.timestampSeconds(),
                observation.stdDevs(),
                1,
                VisionConstants.kAprilTagLayout
                    .getTagPose(observation.id())
                    .get()
                    .minus(new Pose3d(RobotState.getGlobalPose()))
                    .getTranslation()
                    .getNorm()));
      }
    }

    // --------------------------------------------------------------------------------------------------------------------------

    // Optional<VisionFieldPoseEstimate> accepted = Optional.empty();
    // if (maybeMTA.isPresent() != maybeMTB.isPresent()) {
    //   accepted = maybeMTA.isPresent() ? maybeMTA : maybeMTB;
    // } else if (maybeMTA.isPresent() && maybeMTB.isPresent()) {
    //   accepted = Optional.of(fuseEstimates(maybeMTA.get(), maybeMTB.get()));
    // }

    // accepted.ifPresent(
    //     est -> {
    //       Logger.recordOutput("Vision/fusedAccepted", est.getVisionRobotPoseMeters());
    //       state.updateMegatagEstimate(est);
    //     });

    // Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));

    Logger.recordOutput("Vision/objectDetection/isCoralDetected", isCoralDetected());
    if (isCoralDetected()) {
      Logger.recordOutput("Vision/objectDetection/coralTx", getCoralTx());
      Logger.recordOutput("Vision/objectDetection/CoralTy", getCoralTy());
      processCoralDetections();
      CoralPoseTracker.CoralPoseObservation[] coralPoseObservations =
          coralPoseTracker.getObservations();
      Pose3d[] coralPoses = new Pose3d[coralPoseObservations.length];
      for (int i = 0; i < coralPoseObservations.length; i++) {
        //  Logger.recordOutput(
        //       "Vision/objectDetection/CoralPoseObservations/" + i, coralPoseObservations[i]);
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

  // private void logCameraInputs(String prefix, VisionIOLimelight.VisionIOInputs.CameraInputs cam)
  // {
  //   Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget);
  //   Logger.recordOutput(prefix + "/MegatagCount", cam.megatagCount);

  //   if (DriverStation.isDisabled()) {
  //     SmartDashboard.putBoolean(prefix + "/SeesTarget", cam.seesTarget);
  //     SmartDashboard.putNumber(prefix + "/MegatagCount", cam.megatagCount);
  //   }

  //   if (cam.pose3d != null) {
  //     Logger.recordOutput(prefix + "/Pose3d", cam.pose3d);
  //   }

  //   if (cam.megatagPoseEstimate != null) {
  //     Logger.recordOutput(prefix + "/MegatagPoseEstimate",
  // cam.megatagPoseEstimate.fieldToRobot());
  //     Logger.recordOutput(prefix + "/Quality", cam.megatagPoseEstimate.quality());
  //     Logger.recordOutput(prefix + "/AvgTagArea", cam.megatagPoseEstimate.avgTagArea());
  //     Logger.recordOutput(
  //         prefix + "/XStandardDeviations",
  //         cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex]);
  //     Logger.recordOutput(
  //         prefix + "/YStandardDeviations",
  //         cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex]);
  //     Logger.recordOutput(
  //         prefix + "/YawStandardDeviations",
  //         cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex]);
  //     Logger.recordOutput(prefix + "/CameratoTagDist", cam.megatagDistance);
  //   }

  //   // Log MegaTag2 data
  //   Logger.recordOutput(prefix + "/Megatag2Count", cam.megatag2Count);
  //   if (cam.megatag2PoseEstimate != null) {
  //     Logger.recordOutput(
  //         prefix + "/Megatag2PoseEstimate", cam.megatag2PoseEstimate.fieldToRobot());
  //     Logger.recordOutput(prefix + "/Megatag2Quality", cam.megatag2PoseEstimate.quality());
  //     Logger.recordOutput(prefix + "/Megatag2AvgTagArea", cam.megatag2PoseEstimate.avgTagArea());
  //     Logger.recordOutput(
  //         prefix + "/Megatag2XStandardDeviations",
  //         cam.standardDeviations[VisionConstants.kMegatag2XStdDevIndex]);
  //     Logger.recordOutput(
  //         prefix + "/Megatag2YStandardDeviations",
  //         cam.standardDeviations[VisionConstants.kMegatag2YStdDevIndex]);
  //     Logger.recordOutput(
  //         prefix + "/Megatag2YawStandardDeviations",
  //         cam.standardDeviations[VisionConstants.kMegatag2YawStdDevIndex]);
  //     Logger.recordOutput(prefix + "/Megatag2CameratoTagDist", cam.megatag2Distance);
  //   }

  //   if (cam.fiducialObservations != null) {
  //     Logger.recordOutput(prefix + "/FiducialCount", cam.fiducialObservations.length);
  //   }
  // }

  // private Optional<VisionFieldPoseEstimate> processCamera(
  //     VisionIOLimelight.VisionIOInputs.CameraInputs cam, String label, Transform2d robotToCamera)
  // {

  //   String logPrefix = "Vision/" + label;

  //   if (!cam.seesTarget) {
  //     return Optional.empty();
  //   }

  //   Optional<VisionFieldPoseEstimate> estimate = Optional.empty();

  //   // Use MegaTag2 for single tag, MegaTag1 for multiple tags
  //   if (cam.megatag2PoseEstimate != null) {
  //     // MegaTag2 available (single tag in view) - use it with MT2 standard deviations
  //     Optional<VisionFieldPoseEstimate> mt2Estimate =
  //         processMegatag2PoseEstimate(cam.megatag2PoseEstimate, cam, logPrefix);

  //     mt2Estimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(
  //                 logPrefix + "/AcceptedMegatag2Estimate", est.getVisionRobotPoseMeters()));
  //     mt2Estimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(
  //                 logPrefix + "/AcceptedStdDevs", est.getVisionMeasurementStdDevs().getData()));
  //     mt2Estimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(logPrefix + "/AcceptedCameratoTagDist",
  // est.getDistanceToTag()));

  //     if (mt2Estimate.isPresent()) {
  //       estimate = mt2Estimate;
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag2", true);
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
  //       Logger.recordOutput(logPrefix + "/AcceptGyro", false);
  //     } else {
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag2", false);
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
  //       Logger.recordOutput(logPrefix + "/AcceptGyro", false);
  //     }
  //   } else if (cam.megatagPoseEstimate != null) {
  //     // MegaTag1 available (multiple tags in view)
  //     Optional<VisionFieldPoseEstimate> mtEstimate =
  //         processMegatagPoseEstimate(cam.megatagPoseEstimate, cam, logPrefix);

  //     mtEstimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(
  //                 logPrefix + "/AcceptedMegatagEstimate", est.getVisionRobotPoseMeters()));
  //     mtEstimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(
  //                 logPrefix + "/AcceptedStdDevs", est.getVisionMeasurementStdDevs().getData()));
  //     mtEstimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(logPrefix + "/AcceptedCameratoTagDist",
  // est.getDistanceToTag()));

  //     Optional<VisionFieldPoseEstimate> gyroEstimate =
  //         fuseWithGyro(cam.megatagPoseEstimate, cam, logPrefix);

  //     gyroEstimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(
  //                 logPrefix + "/AcceptedMegatagEstimate", est.getVisionRobotPoseMeters()));
  //     gyroEstimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(
  //                 logPrefix + "/AcceptedStdDevs", est.getVisionMeasurementStdDevs().getData()));
  //     gyroEstimate.ifPresent(
  //         est ->
  //             Logger.recordOutput(logPrefix + "/AcceptedCameratoTagDist",
  // est.getDistanceToTag()));

  //     // Prefer Megatag when available
  //     if (mtEstimate.isPresent()) {
  //       estimate = mtEstimate;
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag2", false);
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag", true);
  //       Logger.recordOutput(logPrefix + "/AcceptGyro", false);
  //     } else if (gyroEstimate.isPresent()) {
  //       estimate = gyroEstimate;
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag2", false);
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
  //       Logger.recordOutput(logPrefix + "/AcceptGyro", true);
  //     } else {
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag2", false);
  //       Logger.recordOutput(logPrefix + "/AcceptMegatag", false);
  //       Logger.recordOutput(logPrefix + "/AcceptGyro", false);
  //     }
  //   }

  //   return estimate;
  // }

  // private Optional<VisionFieldPoseEstimate> fuseWithGyro(
  //     MegatagPoseEstimate poseEstimate,
  //     VisionIOLimelight.VisionIOInputs.CameraInputs cam,
  //     String logPrefix) {

  //   if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
  //     return Optional.empty();
  //   }

  //   // Use Megatag directly when 2 or more tags are visible
  //   if (poseEstimate.fiducialIds().length > 1) {
  //     return Optional.empty();
  //   }

  //   // Reject if the robot is yawing rapidly (time‑sync unreliable)
  //   final double kHighYawLookbackS = 0.3;
  //   final double kHighYawVelocityRadS = 5.0;

  //   if (state
  //           .getMaxAbsDriveYawAngularVelocityInRange(
  //               poseEstimate.timestampSeconds() - kHighYawLookbackS,
  //               poseEstimate.timestampSeconds())
  //           .orElse(Double.POSITIVE_INFINITY)
  //       > kHighYawVelocityRadS) {
  //     return Optional.empty();
  //   }

  //   var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
  //   if (priorPose.isEmpty()) {
  //     return Optional.empty();
  //   }

  //   var maybeFieldToTag =
  //       VisionConstants.kAprilTagLayoutReefsOnly.getTagPose(poseEstimate.fiducialIds()[0]);
  //   if (maybeFieldToTag.isEmpty()) {
  //     return Optional.empty();
  //   }

  //   Pose2d fieldToTag =
  //       new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);

  //   Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());

  //   Pose2d posteriorPose =
  //       new Pose2d(
  //           fieldToTag
  //               .getTranslation()
  //               .minus(robotToTag.getTranslation().rotateBy(priorPose.get().getRotation())),
  //           priorPose.get().getRotation());

  //   double xStd = 0.0;
  //   double yStd = 0.0;

  //   if (Constants.currentMode == Mode.REAL) {

  //     xStd =
  //         // cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex]
  //         //     *
  //         VisionConstants
  //                 .kXStdDevCoefficent // semi-random constant tuned so we get a reasonable std
  //             * Math.pow(
  //                 cam.megatagDistance, 2.0) // we are less confident if we are farther from the
  // tag
  //             / Math.pow(
  //                 cam.megatagCount,
  //                 2.0); // we are more confident if we have more tags visible so divide
  //     yStd =
  //         // cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex]
  //         //     *
  //         VisionConstants
  //                 .kXStdDevCoefficent // semi-random constant tuned so we get a reasonable std
  //             * Math.pow(
  //                 cam.megatagDistance, 2.0) // we are less confident if we are farther from the
  // tag
  //             / Math.pow(
  //                 cam.megatagCount,
  //                 2.0); // we are more confident if we have more tags visible so divide

  //   } else {
  //     xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex];
  //     yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex];
  //   }

  //   double xyStd = Math.max(xStd, yStd);

  //   return Optional.of(
  //       new VisionFieldPoseEstimate(
  //           posteriorPose,
  //           poseEstimate.timestampSeconds(),
  //           VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeVariance),
  //           poseEstimate.fiducialIds().length,
  //           cam.megatagDistance));
  // }

  // private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
  //     MegatagPoseEstimate poseEstimate,
  //     VisionIOLimelight.VisionIOInputs.CameraInputs cam,
  //     String logPrefix) {

  //   if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
  //     return Optional.empty();
  //   }

  //   // Single‑tag extra checks
  //   if (poseEstimate.fiducialIds().length < 2 && poseEstimate.fiducialIds().length > 1) {
  //     for (var fiducial : cam.fiducialObservations) {
  //       if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
  //         return Optional.empty();
  //       }
  //     }

  //     if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
  //       return Optional.empty();
  //     }

  //     var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
  //     if (poseEstimate.avgTagArea() < VisionConstants.kTagAreaThresholdForYawCheck
  //         && priorPose.isPresent()) {
  //       double yawDiff =
  //           Math.abs(
  //               MathUtil.angleModulus(
  //                   priorPose.get().getRotation().getRadians()
  //                       - poseEstimate.fieldToRobot().getRotation().getRadians()));

  //       if (yawDiff > Units.degreesToRadians(VisionConstants.kDefaultYawDiffThreshold)) {
  //         return Optional.empty();
  //       }
  //     }
  //   }

  //   if (poseEstimate.fieldToRobot().getTranslation().getNorm()
  //       < VisionConstants.kDefaultNormThreshold) {
  //     return Optional.empty();
  //   }

  //   // if we're not inside the field, we're checking if it's half the width since that's the
  // worst
  //   // case senario
  //   // if (!(poseEstimate.fieldToRobot().getTranslation().getX()
  //   //         < FieldConstants.fieldLength - (Constants.DriveConstants.kBumperWidthInches / 2)
  //   //     || poseEstimate.fieldToRobot().getTranslation().getX()
  //   //         > 0.0 + (Constants.DriveConstants.kBumperWidthInches / 2)
  //   //     || poseEstimate.fieldToRobot().getTranslation().getY()
  //   //         < FieldConstants.fieldWidth - (Constants.DriveConstants.kBumperWidthInches / 2)
  //   //     || poseEstimate.fieldToRobot().getTranslation().getY()
  //   //         > 0.0 + (Constants.DriveConstants.kBumperWidthInches / 2))) {

  //   //   return Optional.empty();
  //   // }

  //   if (Math.abs(cam.pose3d.getZ()) > VisionConstants.kDefaultZThreshold) {
  //     return Optional.empty();
  //   }

  //   // if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForMultipleTagMegatag) {
  //   //   return Optional.empty();
  //   // }

  //   // Exclusive‑tag filtering
  //   // var exclusiveTag = state.getExclusiveTag();
  //   // boolean hasExclusiveId =
  //   //     exclusiveTag.isPresent()
  //   //         && java.util.Arrays.stream(poseEstimate.fiducialIds())
  //   //             .anyMatch(id -> id == exclusiveTag.get());

  //   // if (exclusiveTag.isPresent() && !hasExclusiveId) {
  //   //   return Optional.empty();
  //   // }

  //   var loggedPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
  //   if (loggedPose.isEmpty()) {
  //     return Optional.empty();
  //   }

  //   Pose2d estimatePose = poseEstimate.fieldToRobot();

  //   double xStd = 0.0;
  //   double yStd = 0.0;
  //   double rotStd = 0.0;

  //   if (Constants.currentMode == Mode.REAL) {
  //     xStd =
  //         // cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex]
  //         //     *
  //         VisionConstants
  //                 .kXStdDevCoefficent // semi-random constant tuned so we get a reasonable std
  //             * Math.pow(
  //                 cam.megatagDistance, 2.0) // we are less confident if we are farther from the
  // tag
  //             / Math.pow(
  //                 cam.megatagCount,
  //                 2.0); // we are more confident if we have more tags visible so divide
  //     yStd =
  //         // cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex]
  //         //     *
  //         VisionConstants
  //                 .kXStdDevCoefficent // semi-random constant tuned so we get a reasonable std
  //             * Math.pow(
  //                 cam.megatagDistance, 2.0) // we are less confident if we are farther from the
  // tag
  //             / Math.pow(
  //                 cam.megatagCount,
  //                 2.0); // we are more confident if we have more tags visible so divide
  //     rotStd =
  //         // cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex]
  //         //     *
  //         VisionConstants.thetaStdDevCoefficient
  //             * Math.pow(cam.megatagDistance, 2)
  //             / Math.pow(cam.megatagCount, 2.0);
  //   } else {
  //     double scaleFactor = 1.0 / poseEstimate.quality();
  //     xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex] * scaleFactor;
  //     yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex] * scaleFactor;
  //     rotStd = cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex] * scaleFactor;
  //   }

  //   double xyStd = Math.max(xStd, yStd);
  //   Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

  //   return Optional.of(
  //       new VisionFieldPoseEstimate(
  //           estimatePose,
  //           poseEstimate.timestampSeconds(),
  //           visionStdDevs,
  //           poseEstimate.fiducialIds().length,
  //           cam.megatagDistance));
  // }

  private void processCoralDetections() {
    if (io.getAllCoralTNCs().isPresent()) {
      ArrayList<Pair<Double, Double>> allCoralTNCs =
          io.getAllCoralTNCs().orElseGet(() -> new ArrayList<>()); // io.getAllCoralTNCs().get();
      ArrayList<Pose2d> coralPoses = new ArrayList<>();
      for (Pair<Double, Double> coralTNC : allCoralTNCs) {
        coralPoses.add(calculateCoralPose(coralTNC));
      }
      coralPoseTracker.addObservations(coralPoses);
    }
  }

  private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(
      MegatagPoseEstimate poseEstimate,
      VisionIOLimelight.VisionIOInputs.CameraInputs cam,
      String logPrefix) {

    if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
      return Optional.empty();
    }

    // MegaTag2 is only for single tags, so apply single-tag checks
    for (var fiducial : cam.fiducialObservations) {
      if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
        return Optional.empty();
      }
    }

    if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
      return Optional.empty();
    }

    // var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
    // if (poseEstimate.avgTagArea() < VisionConstants.kTagAreaThresholdForYawCheck
    //     && priorPose.isPresent()) {
    //   double yawDiff =
    //       Math.abs(
    //           MathUtil.angleModulus(
    //               priorPose.get().getRotation().getRadians()
    //                   - poseEstimate.fieldToRobot().getRotation().getRadians()));

    //   if (yawDiff > Units.degreesToRadians(VisionConstants.kDefaultYawDiffThreshold)) {
    //     return Optional.empty();
    //   }
    // }

    if (poseEstimate.fieldToRobot().getTranslation().getNorm()
        < VisionConstants.kDefaultNormThreshold) {
      return Optional.empty();
    }

    // if we're not inside the field, we're checking if it's half the width since that's the worst
    // case senario
    // if (!(poseEstimate.fieldToRobot().getTranslation().getX()
    //         < FieldConstants.fieldLength - (Constants.DriveConstants.kBumperWidthInches / 2)
    //     || poseEstimate.fieldToRobot().getTranslation().getX()
    //         > 0.0 + (Constants.DriveConstants.kBumperWidthInches / 2)
    //     || poseEstimate.fieldToRobot().getTranslation().getY()
    //         < FieldConstants.fieldWidth - (Constants.DriveConstants.kBumperWidthInches / 2)
    //     || poseEstimate.fieldToRobot().getTranslation().getY()
    //         > 0.0 + (Constants.DriveConstants.kBumperWidthInches / 2))) {

    //   return Optional.empty();
    // }

    if (Math.abs(cam.pose3d.getZ()) > VisionConstants.kDefaultZThreshold) {
      return Optional.empty();
    }

    var loggedPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
    if (loggedPose.isEmpty()) {
      return Optional.empty();
    }

    Pose2d estimatePose = poseEstimate.fieldToRobot();

    double xStd = 0.0;
    double yStd = 0.0;
    double rotStd = 0.0;

    if (Constants.currentMode == Mode.REAL) {
      xStd =
          // cam.standardDeviations[VisionConstants.kMegatag2XStdDevIndex]
          //     *
          VisionConstants
                  .kXStdDevCoefficent // semi-random constant tuned so we get a reasonable std
              * Math.pow(
                  cam.megatag2Distance, 2.0) // we are less confident if we are farther from the tag
              / Math.pow(
                  cam.megatag2Count,
                  2.0); // we are more confident if we have more tags visible so divide
      yStd =
          // cam.standardDeviations[VisionConstants.kMegatag2YStdDevIndex]
          //     *
          VisionConstants
                  .kXStdDevCoefficent // semi-random constant tuned so we get a reasonable std
              * Math.pow(
                  cam.megatag2Distance, 2.0) // we are less confident if we are farther from the tag
              / Math.pow(
                  cam.megatag2Count,
                  2.0); // we are more confident if we have more tags visible so divide
      rotStd =
          // cam.standardDeviations[VisionConstants.kMegatag2YawStdDevIndex]
          //     *
          VisionConstants.thetaStdDevCoefficient
              * Math.pow(cam.megatag2Distance, 2)
              / Math.pow(cam.megatag2Count, 2.0);
    } else {
      double scaleFactor = 1.0 / poseEstimate.quality();
      xStd = cam.standardDeviations[VisionConstants.kMegatag2XStdDevIndex] * scaleFactor;
      yStd = cam.standardDeviations[VisionConstants.kMegatag2YStdDevIndex] * scaleFactor;
      rotStd = cam.standardDeviations[VisionConstants.kMegatag2YawStdDevIndex] * scaleFactor;
    }

    double xyStd = Math.max(xStd, yStd);
    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

    return Optional.of(
        new VisionFieldPoseEstimate(
            estimatePose,
            poseEstimate.timestampSeconds(),
            visionStdDevs,
            poseEstimate.fiducialIds().length,
            cam.megatag2Distance));
  }

  public void setUseVision(boolean useVision) {
    this.useVision = useVision;
  }
}
