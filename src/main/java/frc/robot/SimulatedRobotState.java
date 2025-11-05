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
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.util.Color;
import frc.robot.util.MagicVirtualSubsystem;
import frc.robot.util.PoseUtils;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import org.littletonrobotics.junction.Logger;

// Use import "import frc.robot.SimulatedSimulatedRobotState" instead of wpilib's
// SimulatedSimulatedRobotState

public class SimulatedRobotState extends MagicVirtualSubsystem {
  private static final String logRoot = "SimulatedRobotState/";

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = Pose2d.kZero;

  private static ReefTagTracker reefTracker = new ReefTagTracker();
  private static HPSTagTracker hpsTracker = new HPSTagTracker();
  private static BargeTagTracker bargeTracker = new BargeTagTracker();

  private static List<TargetAngleTracker> autoAlignmentTrackers =
      List.of(SimulatedRobotState.hpsTracker, SimulatedRobotState.reefTracker);

  private static Color ledState = Color.kCOOrangePure;

  public static void offerVisionObservation(PoseObservation observation) {
    SimulatedRobotState.poseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getVisionObservations() {
    return SimulatedRobotState.poseObservations;
  }

  public static void updateGlobalPose(Pose2d pose) {
    SimulatedRobotState.globalPose = pose;
  }

  public static Pose2d getGlobalPose() {
    return SimulatedRobotState.globalPose;
  }

  public static Trigger onTeamSide() {
    return new Trigger(
        () ->
            FieldUtils.getAlliance() == Alliance.Blue
                ? getGlobalPose().getX() < FieldConstants.fieldLength / 2.0
                : getGlobalPose().getX() > FieldConstants.fieldLength / 2.0);
  }

  public static Rotation2d getRotationToClosestReef() {
    return SimulatedRobotState.reefTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestHPS() {
    return SimulatedRobotState.hpsTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestBarge() {
    return SimulatedRobotState.bargeTracker.getRotationTarget();
  }

  public static double getDistanceMetersFromClosestHPS() {
    return SimulatedRobotState.hpsTracker.getDistanceMeters();
  }

  public static void setLedState(Color state) {
    SimulatedRobotState.ledState = state;
  }

  public static Color getLedState() {
    return SimulatedRobotState.ledState;
  }

  public static Trigger humanPlayerShouldThrow() {
    return new Trigger(
        () ->
            PoseUtils.getPerpendicularError(
                    SimulatedRobotState.getGlobalPose(),
                    FieldUtils.getClosestHPSTag().pose().toPose2d())
                < 0.5);
  }

  public static TargetAngleTracker getClosestAlignmentTracker() {
    return autoAlignmentTrackers.stream()
        .reduce((a, b) -> a.getDistanceMeters() < b.getDistanceMeters() ? a : b)
        .get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Robot Pose", getGlobalPose());

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
      Logger.recordOutput(
          calcLogRoot + "Distance", SimulatedRobotState.hpsTracker.getDistanceMeters());
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

  public Pose2d getLatestFieldToRobot() {
    return getGlobalPose();
  }

  @Override
  public void simulationPeriodic() {}
}
