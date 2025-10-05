package frc.robot;

import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.littletonrobotics.junction.Logger;

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

// Use import "import frc.robot.RobotState" instead of wpilib's RobotState

public class RobotState extends MagicVirtualSubsystem {

  public enum ReefSide{
    A, B, C, D, E, F, NONE
  }
  public enum CoralBranch{
    RIGHT, LEFT, NONE
  }
  public enum BranchLevel{
    ONE, TWO, THREE, FOUR, NONE
  }
  public enum AlgaeLevel{
    BARGE, L2, L1, PROCESSOR, NONE
  }

  class ScorePosition{
    private ReefSide reefSide;
    private CoralBranch coralBranch;
    private BranchLevel branchLevel;
    private AlgaeLevel algaeLevel;
    public ScorePosition(){
      this.reefSide = ReefSide.NONE;
      this.coralBranch = CoralBranch.NONE;
      this.branchLevel = BranchLevel.NONE;
      this.algaeLevel = AlgaeLevel.NONE;
    }
    public ReefSide getReefSide(){
      return reefSide;
    }
    public CoralBranch getCoralBranch(){
      return coralBranch;
    }
    public BranchLevel getBranchLevel(){
      return branchLevel;
    }
    public AlgaeLevel getAlgaeLevel(){
      return algaeLevel;
    }
    public void setReefSide(ReefSide reefSide){
      this.reefSide = reefSide;
    }
    public void setCoralBranch(CoralBranch coralBranch){
      this.coralBranch = coralBranch;
    }
    public void setBranchLevel(BranchLevel branchLevel){
      this.branchLevel = branchLevel;
    }
    public void setAlgaeLevel(AlgaeLevel algaeLevel){
      this.algaeLevel = algaeLevel;
    }
  }

  private static ScorePosition storedScorePosition;

  public static ScorePosition getStoredScorePosition(){
    return storedScorePosition;
  }

  public static SuperstructureState getSuperstructureStateAim(){
    switch (getStoredScorePosition().getBranchLevel()) {
      case ONE: 
        return SuperstructureState.L1_PIVOT;
      case TWO: 
        return SuperstructureState.L2_AIM;
      case THREE: 
        return SuperstructureState.L3_AIM;
      case FOUR: 
        return SuperstructureState.L4_AIM;
      case NONE: 
        return SuperstructureState.NONE;
      default:
        return SuperstructureState.NONE;
    }
  }
  public static SuperstructureState getSuperstructureStateScore(){
    switch (getStoredScorePosition().getBranchLevel()) {
      case ONE: 
        return SuperstructureState.L1_SCORE;
      case TWO: 
        return SuperstructureState.L2_SCORE;
      case THREE: 
        return SuperstructureState.L3_SCORE;
      case FOUR: 
        return SuperstructureState.L4_SCORE;
      case NONE: 
        return SuperstructureState.NONE;
      default:
        return SuperstructureState.NONE;
    }
  }



  private static final String logRoot = "RobotState/";

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = new Pose2d();

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
