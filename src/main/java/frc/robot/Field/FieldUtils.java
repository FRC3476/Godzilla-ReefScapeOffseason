package frc.robot.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Field.FieldConstants.AprilTagStruct;
import frc.robot.RobotState;
import java.util.List;
import java.util.function.DoubleSupplier;

public class FieldUtils {
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
  }

  public static boolean isBlueAlliance() {
    return FieldUtils.getAlliance() == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return FieldUtils.getAlliance() == Alliance.Red;
  }

  public static double getFlipped() {
    return FieldUtils.isRedAlliance() ? -1 : 1;
  }

  public static ReefFace getClosestReef() {
    List<ReefFace> reefTags =
        FieldUtils.isBlueAlliance() ? FieldConstants.blueReefTags : FieldConstants.redReefTags;
    Translation2d robotTranslation = RobotState.getGlobalPose().getTranslation();

    ReefFace closestReef =
        reefTags.stream()
            .reduce(
                (ReefFace reef1, ReefFace reef2) ->
                    robotTranslation.getDistance(
                                reef1.tag.pose().getTranslation().toTranslation2d())
                            < robotTranslation.getDistance(
                                reef2.tag.pose().getTranslation().toTranslation2d())
                        ? reef1
                        : reef2)
            .get();
    return closestReef;
  }

  public static ReefPole getClosestReefPole() {
    List<ReefFace> reefTags =
        FieldUtils.isBlueAlliance() ? FieldConstants.blueReefTags : FieldConstants.redReefTags;
    Translation2d robotTranslation = RobotState.getGlobalPose().getTranslation();

    // Collect all reef poles from all reef faces
    List<ReefPole> allReefPoles =
        reefTags.stream()
            .flatMap(reefFace -> List.of(reefFace.leftPole, reefFace.rightPole).stream())
            .toList();

    ReefPole closestReefPole =
        allReefPoles.stream()
            .reduce(
                (ReefPole pole1, ReefPole pole2) ->
                    robotTranslation.getDistance(pole1.getPose().getTranslation())
                            < robotTranslation.getDistance(pole2.getPose().getTranslation())
                        ? pole1
                        : pole2)
            .get();

    return closestReefPole;
  }

  public static ReefPole getChosenReefPole(DoubleSupplier sideSelect) {
    List<ReefFace> reefTags =
        FieldUtils.isBlueAlliance() ? FieldConstants.blueReefTags : FieldConstants.redReefTags;
    Translation2d robotTranslation = RobotState.getGlobalPose().getTranslation();

    // Collect all reef poles from all reef faces
    List<ReefPole> allReefPoles =
        reefTags.stream()
            .flatMap(reefFace -> List.of(reefFace.leftPole, reefFace.rightPole).stream())
            .toList();

    ReefPole closestReefPole =
        allReefPoles.stream()
            .reduce(
                (ReefPole pole1, ReefPole pole2) ->
                    robotTranslation.getDistance(pole1.getPose().getTranslation())
                            < robotTranslation.getDistance(pole2.getPose().getTranslation())
                        ? pole1
                        : pole2)
            .get();

    ReefPole chosenReefPole;

    if (sideSelect.getAsDouble() < -0.5) {
      chosenReefPole = getClosestReef().leftPole;
    } else if (sideSelect.getAsDouble() > 0.5) chosenReefPole = getClosestReef().rightPole;
    else {
      chosenReefPole = closestReefPole;
    }

    return chosenReefPole;
  }

  public static AprilTagStruct getClosestHPSTag() {
    List<AprilTagStruct> hpsTags =
        FieldUtils.isBlueAlliance() ? FieldConstants.blueHPSTags : FieldConstants.redHPSTags;

    Translation2d robotTranslation = RobotState.getGlobalPose().getTranslation();

    AprilTagStruct closestTag =
        hpsTags.stream()
            .reduce(
                (AprilTagStruct tag1, AprilTagStruct tag2) ->
                    robotTranslation.getDistance(tag1.pose().getTranslation().toTranslation2d())
                            < robotTranslation.getDistance(
                                tag2.pose().getTranslation().toTranslation2d())
                        ? tag1
                        : tag2)
            .get();

    return closestTag;
  }

  public static boolean isOnAllianceSide() {
    double robotX = RobotState.getGlobalPose().getTranslation().getX();
    if (FieldUtils.isBlueAlliance()) {
      return robotX < FieldConstants.halfFieldLength;
    } else {
      return robotX > FieldConstants.halfFieldLength;
    }
  }

  public static boolean isOnRedSide() {
    double robotX = RobotState.getGlobalPose().getTranslation().getX();
    return robotX > FieldConstants.halfFieldLength;
  }

  public static boolean isOnBlueSide() {
    double robotX = RobotState.getGlobalPose().getTranslation().getX();
    return robotX < FieldConstants.halfFieldLength;
  }

  public static boolean facingBarge() {
    return (FieldUtils.isOnAllianceSide() ? 1 : -1)
            * (FieldUtils.isRedAlliance() ? -1 : 1)
            * RobotState.getGlobalPose().getRotation().getCos()
        > 0;
  }

  public static AprilTagStruct getBargeTag() {
    return FieldUtils.isBlueAlliance() ? FieldConstants.blueBarge : FieldConstants.redBarge;
  }

  public static boolean isInsideField(Pose2d pose) {
    return isInsideField(pose, 0.0);
  }

  public static boolean isInsideField(Pose2d pose, double bufferMeters) {
    return pose.getX() >= bufferMeters
        && pose.getX() <= FieldConstants.fieldLength - bufferMeters
        && pose.getY() >= bufferMeters
        && pose.getY() <= FieldConstants.fieldWidth - bufferMeters;
  }
}
