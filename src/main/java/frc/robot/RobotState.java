package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Field.FieldConstants;
import frc.robot.Field.FieldConstants.AprilTagStruct;
import frc.robot.Field.FieldUtils;
import frc.robot.Field.varc.BargeTagTracker;
import frc.robot.Field.varc.HPSTagTracker;
import frc.robot.Field.varc.ReefTagTracker;
import frc.robot.Field.varc.TargetAngleTracker;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.MagicVirtualSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.PoseUtils;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

// Use import "import frc.robot.RobotState" instead of wpilib's RobotState

public class RobotState extends MagicVirtualSubsystem {

  public enum ReefSide {
    A,
    B,
    C,
    D,
    E,
    F,
    NONE
  }

  public enum CoralBranch {
    RIGHT,
    LEFT,
    NONE
  }

  public enum ScoreLevel {
    L1,
    L2,
    L3,
    L4,
    BARGE, PROCESSOR, NONE
  }

  public enum AlgaeIntake {
    L2_ALGAE,
    L1_ALGAE,
    NONE
  }


  class ScorePosition{
    private ReefSide reefSide;
    private CoralBranch coralBranch;
    private ScoreLevel scoreLevel;
    public ScorePosition(){
      this.reefSide = ReefSide.NONE;
      this.coralBranch = CoralBranch.NONE;
      this.scoreLevel = ScoreLevel.NONE;
    }

    public ReefSide getReefSide() {
      return reefSide;
    }

    public CoralBranch getCoralBranch() {
      return coralBranch;
    }

    public ScoreLevel getScoreLevel() {
      return scoreLevel;
    }
    public void setReefSide(ReefSide reefSide){
      this.reefSide = reefSide;
    }
    public void setBranchSide(CoralBranch coralBranch){
      this.coralBranch = coralBranch;
    }
    public void setScoreLevel(ScoreLevel scoreLevel){
      this.scoreLevel = scoreLevel;
    }
  }

  private ScorePosition storedScorePosition;

  public ScorePosition getStoredScorePosition(){
    return storedScorePosition;
  }

  public SuperstructureState getSuperstructureScoreAimState(){
    switch (getStoredScorePosition().getScoreLevel()) {
      case L1:
        return SuperstructureState.L1_PIVOT;
      case L2:
        return SuperstructureState.L2_AIM;
      case L3:
        return SuperstructureState.L3_AIM;
      case L4:
        return SuperstructureState.L4_AIM;
      case BARGE:
        return SuperstructureState.BARGE_AIM_BACKWARD;
      case PROCESSOR:
        return SuperstructureState.PROCESSOR_AIM;
      default:
        return SuperstructureState.NONE;
    }
  }

  private static final String logRoot = "RobotState/";

  private static SuperstructureState currenState = SuperstructureState.NONE;

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

  private static Pose2d globalPose = Pose2d.kZero;

  private static ReefTagTracker reefTracker = new ReefTagTracker();
  private static HPSTagTracker hpsTracker = new HPSTagTracker();
  private static BargeTagTracker bargeTracker = new BargeTagTracker();

  private static HashMap<AprilTagStruct, SuperstructureState> algaeDescoreMap = new HashMap(
    
  );

  private static boolean hasAlgae = false;

  public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
    this.visionEstimateConsumer = visionEstimateConsumer;
    fieldToRobot.addSample(0.0, MathHelpers.kPose2dZero);
    driveYawAngularVelocity.addSample(0.0, 0.0);
    storedScorePosition = new ScorePosition();
  }

  private static List<TargetAngleTracker> autoAlignmentTrackers =
      List.of(RobotState.hpsTracker, RobotState.reefTracker);

  private static LedState ledState = LedState.kCOOrange;

  public static void offerVisionObservation(PoseObservation observation) {
    RobotState.poseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getVisionObservations() {
    return RobotState.poseObservations;
  }

  public static void updateGlobalPose(Pose2d pose) {
    RobotState.globalPose = pose;
  }

  public static Pose2d getGlobalPose() {
    return RobotState.globalPose;
  }

  public static void setSuperstructureState(SuperstructureState state) {
    currenState = state;
  }

  public static SuperstructureState getSuperstructureState() {
    return currenState;
  }

  public static Trigger onTeamSide() {
    return new Trigger(
        () ->
            FieldUtils.getAlliance() == Alliance.Blue
                ? getGlobalPose().getX() < FieldConstants.fieldLength / 2.0
                : getGlobalPose().getX() > FieldConstants.fieldLength / 2.0);
  }

  public static Rotation2d getRotationToClosestReef() {
    return RobotState.reefTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestHPS() {
    return RobotState.hpsTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestBarge() {
    return RobotState.bargeTracker.getRotationTarget();
  }

  public static double getDistanceMetersFromClosestHPS() {
    return RobotState.hpsTracker.getDistanceMeters();
  }

  public static Trigger humanPlayerShouldThrow() {
    return new Trigger(
        () ->
            PoseUtils.getPerpendicularError(
                    RobotState.getGlobalPose(), FieldUtils.getClosestHPSTag().pose().toPose2d())
                < 0.5);
  }

  public static TargetAngleTracker getClosestAlignmentTracker() {
    return autoAlignmentTrackers.stream()
        .reduce((a, b) -> a.getDistanceMeters() < b.getDistanceMeters() ? a : b)
        .get();
  }

  public static boolean hasAlgae() {
    return hasAlgae;
  }

  public static void setHasAlgae(boolean input) {
    hasAlgae = input;
  }

  public static final double LOOKBACK_TIME = 1.0;

  // State of robot.

  // Kinematic Frames
  // Robot's pose in field coordinates over time
  private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
      ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
  // Current robot-relative chassis speeds (measured from encoders)
  private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  // Current field-relative chassis speeds (measured from encoders)
  private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  // Desired robot-relative chassis speeds (set by control systems)
  private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  // Desired field-relative chassis speeds (set by control systems)
  private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());

  private final AtomicInteger iteration = new AtomicInteger(0);

  private double lastUsedMegatagTimestamp = 0;
  private Pose2d lastUsedMegatagPose = Pose2d.kZero;
  private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

  private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> accelX =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> accelY =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

  private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

  private double autoStartTime;

  private Optional<Pose2d> trajectoryTargetPose = Optional.empty();
  private Optional<Pose2d> trajectoryCurrentPose = Optional.empty();

  public void setAutoStartTime(double timestamp) {
    autoStartTime = timestamp;
  }

  public double getAutoStartTime() {
    return autoStartTime;
  }

  public void enablePathCancel() {
    enablePathCancel.set(true);
  }

  public void disablePathCancel() {
    enablePathCancel.set(false);
  }

  public boolean getPathCancel() {
    return enablePathCancel.get();
  }

  public void addOdometryMeasurement(double timestamp, Pose2d pose) {
    fieldToRobot.addSample(timestamp, pose);
  }

  public void incrementIterationCount() {
    iteration.incrementAndGet();
  }

  public int getIteration() {
    return iteration.get();
  }

  public IntSupplier getIterationSupplier() {
    return () -> getIteration();
  }

  public void addDriveMotionMeasurements(
      double timestamp,
      double angularRollRadsPerS,
      double angularPitchRadsPerS,
      double angularYawRadsPerS,
      double pitchRads,
      double rollRads,
      double accelX,
      double accelY,
      ChassisSpeeds desiredRobotRelativeChassisSpeeds,
      ChassisSpeeds desiredFieldRelativeSpeeds,
      ChassisSpeeds measuredSpeeds,
      ChassisSpeeds measuredFieldRelativeSpeeds,
      ChassisSpeeds fusedFieldRelativeSpeeds) {
    this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
    this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
    this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
    this.drivePitchRads.addSample(timestamp, pitchRads);
    this.driveRollRads.addSample(timestamp, rollRads);
    this.accelY.addSample(timestamp, accelY);
    this.accelX.addSample(timestamp, accelX);
    this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
    this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
    this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
    this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
    this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
  }

  public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
    return fieldToRobot.getLatest();
  }

  /**
   * Predicts robot's future pose based on current velocity.
   *
   * @param lookaheadTimeS How far ahead to predict (seconds)
   * @return Predicted pose
   */
  public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
    var maybeFieldToRobot = getLatestFieldToRobot();
    Pose2d fieldToRobot =
        maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
    var delta = getLatestRobotRelativeChassisSpeed();
    delta = delta.times(lookaheadTimeS);
    return fieldToRobot.exp(
        new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
  }

  /**
   * Like getPredictedFieldToRobot but caps negative velocities to zero. Used for non-holonomic path
   * planning.
   */
  public Pose2d getPredictedCappedFieldToRobot(double lookaheadTimeS) {
    var maybeFieldToRobot = getLatestFieldToRobot();
    Pose2d fieldToRobot =
        maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
    var delta = getLatestRobotRelativeChassisSpeed();
    delta = delta.times(lookaheadTimeS);
    return fieldToRobot.exp(
        new Twist2d(
            Math.max(0.0, delta.vxMetersPerSecond),
            Math.max(0.0, delta.vyMetersPerSecond),
            delta.omegaRadiansPerSecond));
  }

  public Optional<Pose2d> getFieldToRobot(double timestamp) {
    return fieldToRobot.getSample(timestamp);
  }

  public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
    return measuredFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
    return measuredRobotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
    return desiredRobotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
    return desiredFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
    return fusedFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
    var speeds = getLatestRobotRelativeChassisSpeed();
    speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
    return speeds;
  }

  public static void setLedState(LedState state) {
    RobotState.ledState = state;
  }

  public static LedState getLedState() {
    return RobotState.ledState;
  }

  private Optional<Double> getMaxAbsValueInRange(
      ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
    var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
    var max = submap.stream().max(Double::compare);
    var min = submap.stream().min(Double::compare);
    if (max.isEmpty() || min.isEmpty()) return Optional.empty();
    if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
    else return min;
  }

  public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
    // Gyro yaw rate not set in sim.
    if (Robot.isReal()) return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
    return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
  }

  public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(
      double minTime, double maxTime) {
    return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
  }

  public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(double minTime, double maxTime) {
    return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
  }

  public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
    lastUsedMegatagTimestamp = megatagEstimate.getTimestampSeconds();
    lastUsedMegatagPose = megatagEstimate.getVisionRobotPoseMeters();
    visionEstimateConsumer.accept(megatagEstimate);
  }

  public double lastUsedMegatagTimestamp() {
    return lastUsedMegatagTimestamp;
  }

  public Pose2d lastUsedMegatagPose() {
    return lastUsedMegatagPose;
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
  }

  public void updateLogger() {
    if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/YawAngularVelocity",
          this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
    }
    if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/RollAngularVelocity",
          this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
    }
    if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/PitchAngularVelocity",
          this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
    }
    if (this.drivePitchRads.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/PitchRads", this.drivePitchRads.getInternalBuffer().lastEntry().getValue());
    }
    if (this.driveRollRads.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/RollRads", this.driveRollRads.getInternalBuffer().lastEntry().getValue());
    }
    if (this.accelX.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/AccelX", this.accelX.getInternalBuffer().lastEntry().getValue());
    }
    if (this.accelY.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/AccelY", this.accelY.getInternalBuffer().lastEntry().getValue());
    }
    Logger.recordOutput(
        "RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeed());
    Logger.recordOutput(
        "RobotState/DesiredChassisSpeedRobotFrame", getLatestDesiredRobotRelativeChassisSpeeds());
    Logger.recordOutput(
        "RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
    Logger.recordOutput(
        "RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeed());

    // // Add mechanism logging
    // Logger.recordOutput("RobotState/ElevatorHeightMeters", getElevatorHeightMeters());
    // Logger.recordOutput("RobotState/WristRadians", getWristRadians());
    // Logger.recordOutput("RobotState/IntakeRollerRotations", getIntakeRollerRotations());
    // Logger.recordOutput("RobotState/CoralRollerRotations", getClawRollerRotations());

    // Add LED state logging
    LedState currentLEDState = getLedState();
    Logger.recordOutput(
        "RobotState/LEDState",
        String.format(
            "R:%d G:%d B:%d", currentLEDState.red, currentLEDState.green, currentLEDState.blue));
  }

  private final AtomicReference<Optional<Integer>> exclusiveTag =
      new AtomicReference<>(Optional.empty());

  public void setExclusiveTag(int id) {
    exclusiveTag.set(Optional.of(id));
  }

  public void clearExclusiveTag() {
    exclusiveTag.set(Optional.empty());
  }

  public Optional<Integer> getExclusiveTag() {
    return exclusiveTag.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Robot Pose", getGlobalPose());
    Logger.recordOutput("Coral State Tracker", CoralStateTracker.getCurrentPosition());

    updateLogger();

    {
      reefTracker.update();

      String calcLogRoot = logRoot + "Reef/";
      Logger.recordOutput(calcLogRoot + "ClosestTag", FieldUtils.getClosestReef().tag);
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", reefTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", reefTracker.getRotationTarget().getRadians());
      Logger.recordOutput(calcLogRoot + "Left Pole", FieldUtils.getClosestReef().leftPole);
      Logger.recordOutput(calcLogRoot + "Right Pole", FieldUtils.getClosestReef().rightPole);
    }

    {
      hpsTracker.update();

      String calcLogRoot = logRoot + "HPS/";
      Logger.recordOutput(calcLogRoot + "Closest Tag", FieldUtils.getClosestHPSTag());
      Logger.recordOutput(calcLogRoot + "Distance", RobotState.hpsTracker.getDistanceMeters());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    }

    {
      bargeTracker.update();

      String calcLogRoot = logRoot + "Barge/";
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    }

    {
      String calcLogRoot = logRoot + "ClosestAlignment/";
      Logger.recordOutput(
          calcLogRoot + "Type", getClosestAlignmentTracker().getClass().getSimpleName());
    }
  }

  @Override
  public void simulationPeriodic() {}
}
