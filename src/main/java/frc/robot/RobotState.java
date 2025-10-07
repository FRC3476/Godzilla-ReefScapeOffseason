package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Field.FieldConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.Field.varc.BargeTagTracker;
import frc.robot.Field.varc.HPSTagTracker;
import frc.robot.Field.varc.ReefTagTracker;
import frc.robot.Field.varc.TargetAngleTracker;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.util.MagicVirtualSubsystem;
import frc.robot.util.PoseUtils;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
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
    NONE,
    BARGE,
  }

  public enum AlgaeIntake {
    L2_ALGAE,
    L1_ALGAE,
    NONE
  }

  static class ScorePosition {
    private ReefSide reefSide;
    private CoralBranch coralBranch;
    private ScoreLevel scoreLevel;
    private AlgaeIntake algaeIntake;

    public ScorePosition() {
      this.reefSide = ReefSide.NONE;
      this.coralBranch = CoralBranch.NONE;
      this.scoreLevel = ScoreLevel.NONE;
      this.algaeIntake = AlgaeIntake.NONE;
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

    public AlgaeIntake getAlgaeIntake() {
      return algaeIntake;
    }

    public void setReefSide(ReefSide reefSide) {
      this.reefSide = reefSide;
    }

    public void setCoralBranch(CoralBranch coralBranch) {
      this.coralBranch = coralBranch;
    }

    public void setScoreLevel(ScoreLevel branchLevel) {
      this.scoreLevel = branchLevel;
    }

    public void setAlgaeIntake(AlgaeIntake algaeIntake) {
      this.algaeIntake = algaeIntake;
    }
  }

  private static ScorePosition storedScorePosition = new ScorePosition();

  public static ScorePosition getStoredScorePosition() {
    return storedScorePosition;
  }

  public static SuperstructureState getSuperstructureScoreStates() {
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
      case NONE:
        return SuperstructureState.NONE;
      default:
        return SuperstructureState.NONE;
    }
  }

  public static SuperstructureState getSuperstructureAlgaeDescoreStates() {
    switch (getStoredScorePosition().getAlgaeIntake()) {
      case L1_ALGAE:
        return SuperstructureState.ALGAE_LOW_INTAKE;
      case L2_ALGAE:
        return SuperstructureState.ALGAE_HIGH_INTAKE;
      case NONE:
        return SuperstructureState.NONE;
      default:
        return SuperstructureState.NONE;
    }
  }

  private static final String logRoot = "RobotState/";

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = Pose2d.kZero;

  private static ReefTagTracker reefTracker = new ReefTagTracker();
  private static HPSTagTracker hpsTracker = new HPSTagTracker();
  private static BargeTagTracker bargeTracker = new BargeTagTracker();

  private static boolean hasAlgae = false;

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

  public static void setLedState(LedState state) {
    RobotState.ledState = state;
  }

  public static LedState getLedState() {
    return RobotState.ledState;
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

  @Override
  public void periodic() {
    Logger.recordOutput("Robot Pose", getGlobalPose());

    // {
    //   reefTracker.update();

    //   String calcLogRoot = logRoot + "Reef/";
    //   Logger.recordOutput(calcLogRoot + "ClosestTag", FieldUtils.getClosestReef().tag);
    //   Logger.recordOutput(
    //       calcLogRoot + "TargetAngleDeg", reefTracker.getRotationTarget().getDegrees());
    //   Logger.recordOutput(
    //       calcLogRoot + "TargetAngleRad", reefTracker.getRotationTarget().getRadians());
    //   Logger.recordOutput(calcLogRoot + "Left Pole", FieldUtils.getClosestReef().leftPole);
    //   Logger.recordOutput(calcLogRoot + "Right Pole", FieldUtils.getClosestReef().rightPole);
    // }

    // {
    //   hpsTracker.update();

    //   String calcLogRoot = logRoot + "HPS/";
    //   Logger.recordOutput(calcLogRoot + "Closest Tag", FieldUtils.getClosestHPSTag());
    //   Logger.recordOutput(calcLogRoot + "Distance", RobotState.hpsTracker.getDistanceMeters());
    //   Logger.recordOutput(
    //       calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
    //   Logger.recordOutput(
    //       calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    // }

    // {
    //   bargeTracker.update();

    //   String calcLogRoot = logRoot + "Barge/";
    //   Logger.recordOutput(
    //       calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
    //   Logger.recordOutput(
    //       calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    // }

    // {
    //   String calcLogRoot = logRoot + "ClosestAlignment/";
    //   Logger.recordOutput(
    //       calcLogRoot + "Type", getClosestAlignmentTracker().getClass().getSimpleName());
    // }
  }

  @Override
  public void simulationPeriodic() {}
}
