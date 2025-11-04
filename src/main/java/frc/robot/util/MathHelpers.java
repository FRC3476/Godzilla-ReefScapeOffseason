package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class MathHelpers {

  public static final Pose2d pose2dFromRotation(Rotation2d rotation) {
    return new Pose2d(Translation2d.kZero, rotation);
  }

  public static final Pose2d pose2dFromTranslation(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  public static final Transform2d transform2dFromRotation(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  public static final Transform2d transform2dFromTranslation(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  public static double reverseInterpolate(
      Translation2d query, Translation2d start, Translation2d end) {
    Translation2d segment = end.minus(start);
    Translation2d queryToStart = query.minus(start);

    double segmentLengthSqr = segment.getX() * segment.getX() + segment.getY() * segment.getY();

    if (segmentLengthSqr == 0.0) { // start and end are the same point
      return 0.0;
    }

    return dotProduct(queryToStart, segment) / segmentLengthSqr;
  }

  public static double distanceToLineSegment(
      Translation2d query, Translation2d start, Translation2d end) {
    double t = reverseInterpolate(query, start, end);
    if (t < 0.0) { // closest point is before start
      return query.getDistance(start);
    } else if (t > 1.0) { // closest point is after end
      return query.getDistance(end);
    } else { // closest point is within the segment
      Translation2d segment = end.minus(start);
      Translation2d closestPoint = start.plus(segment.times(t));
      return query.getDistance(closestPoint);
    }
  }

  public static double perpendicularDistanceToLine(
      Translation2d query, Translation2d start, Translation2d end) {
    double t = reverseInterpolate(query, start, end);
    Translation2d segment = end.minus(start);
    Translation2d closestPoint = start.plus(segment.times(t));
    return query.getDistance(closestPoint);
  }

  public static double distanceToPointInDirection(Pose2d first, Translation2d second) {
    return distanceToPointInDirection(first.getTranslation(), second, first.getRotation());
  }

  public static double distanceToPointInDirection(
      Translation2d first, Translation2d second, Rotation2d direction) {
    return dotProduct(
        second.minus(first), new Translation2d(direction.getCos(), direction.getSin()));
  }

  public static double dotProduct(Translation2d first, Translation2d second) {
    return first.getX() * second.getX() + first.getY() * second.getY();
  }

  public static double dotProduct(Translation3d first, Translation3d second) {
    return first.getX() * second.getX()
        + first.getY() * second.getY()
        + first.getZ() * second.getZ();
  }

  public static Translation2d projectedOnto(
      Translation2d toProject, Translation2d projectDirection) {
    double normSq = projectDirection.getNorm() * projectDirection.getNorm();
    if (normSq == 0) return Translation2d.kZero;
    return projectDirection.times(dotProduct(toProject, projectDirection) / normSq);
  }

  public static Translation3d projectedOnto(
      Translation3d toProject, Translation3d projectDirection) {
    double normSq = projectDirection.getNorm() * projectDirection.getNorm();
    if (normSq == 0) return Translation3d.kZero;
    return projectDirection.times(dotProduct(toProject, projectDirection) / normSq);
  }

  public static int crossProductDirection(Translation2d first, Translation2d second) {
    // useful for calculating flipping signs for some oriented actions
    double cross = first.getX() * second.getY() - first.getY() * second.getX();
    if (cross > 0) return 1;
    if (cross < 0) return -1;
    return 0;
  }

  public static Translation3d crossProduct(Translation3d first, Translation3d second) {
    return new Translation3d(
        first.getY() * second.getZ() - first.getZ() * second.getY(),
        first.getZ() * second.getX() - first.getX() * second.getZ(),
        first.getX() * second.getY() - first.getY() * second.getX());
  }
}
