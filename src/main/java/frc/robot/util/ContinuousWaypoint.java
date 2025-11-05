package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/**
 * Represents a waypoint in a continuous path following routine.
 *
 * <p>Based on Team 2056's continuous move approach, waypoints can be configured with different
 * switching distances (1.5" to 48") to control when the robot advances to the next target.
 */
public class ContinuousWaypoint {
  private final Pose2d targetPose;
  private final double switchingDistanceMeters;
  private final boolean isFinalWaypoint;

  /**
   * Creates a new continuous waypoint.
   *
   * @param targetPose The target pose for this waypoint
   * @param switchingDistanceMeters Distance threshold to switch to next waypoint (meters)
   * @param isFinalWaypoint Whether this is the final waypoint in the path
   */
  public ContinuousWaypoint(
      Pose2d targetPose, double switchingDistanceMeters, boolean isFinalWaypoint) {
    this.targetPose = targetPose;
    this.switchingDistanceMeters = switchingDistanceMeters;
    this.isFinalWaypoint = isFinalWaypoint;
  }

  /**
   * Creates a new continuous waypoint with default switching distance.
   *
   * @param targetPose The target pose for this waypoint
   * @param isFinalWaypoint Whether this is the final waypoint in the path
   */
  public ContinuousWaypoint(Pose2d targetPose, boolean isFinalWaypoint) {
    this(
        targetPose,
        DriveConstants.CONTINUOUS_PATH_DEFAULT_SWITCHING_DISTANCE_METERS,
        isFinalWaypoint);
  }

  /**
   * Creates an intermediate waypoint (not final) with default switching distance.
   *
   * @param targetPose The target pose for this waypoint
   * @return A new intermediate waypoint
   */
  public static ContinuousWaypoint intermediate(Pose2d targetPose) {
    return new ContinuousWaypoint(targetPose, false);
  }

  /**
   * Creates an intermediate waypoint (not final) with custom switching distance.
   *
   * @param targetPose The target pose for this waypoint
   * @param switchingDistanceMeters Distance threshold to switch to next waypoint (meters)
   * @return A new intermediate waypoint
   */
  public static ContinuousWaypoint intermediate(Pose2d targetPose, double switchingDistanceMeters) {
    return new ContinuousWaypoint(targetPose, switchingDistanceMeters, false);
  }

  /**
   * Creates an intermediate waypoint with switching distance specified in inches.
   *
   * @param targetPose The target pose for this waypoint
   * @param switchingDistanceInches Distance threshold to switch to next waypoint (inches)
   * @return A new intermediate waypoint
   */
  public static ContinuousWaypoint intermediateInches(
      Pose2d targetPose, double switchingDistanceInches) {
    return new ContinuousWaypoint(targetPose, Units.inchesToMeters(switchingDistanceInches), false);
  }

  /**
   * Creates the final waypoint in a path.
   *
   * @param targetPose The target pose for this waypoint
   * @return A new final waypoint
   */
  public static ContinuousWaypoint finalWaypoint(Pose2d targetPose) {
    return new ContinuousWaypoint(targetPose, 0.0, true); // Switching distance irrelevant for final
  }

  /**
   * Creates a waypoint from x, y coordinates with default heading (0 degrees) and default switching
   * distance.
   *
   * @param x X coordinate in meters
   * @param y Y coordinate in meters
   * @param isFinalWaypoint Whether this is the final waypoint
   * @return A new waypoint
   */
  public static ContinuousWaypoint fromXY(double x, double y, boolean isFinalWaypoint) {
    return new ContinuousWaypoint(new Pose2d(x, y, Rotation2d.kZero), isFinalWaypoint);
  }

  /**
   * Creates a waypoint from x, y coordinates and rotation with default switching distance.
   *
   * @param x X coordinate in meters
   * @param y Y coordinate in meters
   * @param rotation Target rotation
   * @param isFinalWaypoint Whether this is the final waypoint
   * @return A new waypoint
   */
  public static ContinuousWaypoint fromXYRotation(
      double x, double y, Rotation2d rotation, boolean isFinalWaypoint) {
    return new ContinuousWaypoint(new Pose2d(x, y, rotation), isFinalWaypoint);
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public double getSwitchingDistanceMeters() {
    return switchingDistanceMeters;
  }

  public double getSwitchingDistanceInches() {
    return Units.metersToInches(switchingDistanceMeters);
  }

  public boolean isFinalWaypoint() {
    return isFinalWaypoint;
  }

  /**
   * Creates a copy of this waypoint with a different switching distance.
   *
   * @param newSwitchingDistanceMeters New switching distance in meters
   * @return A new waypoint with updated switching distance
   */
  public ContinuousWaypoint withSwitchingDistance(double newSwitchingDistanceMeters) {
    return new ContinuousWaypoint(targetPose, newSwitchingDistanceMeters, isFinalWaypoint);
  }

  /**
   * Creates a copy of this waypoint with switching distance specified in inches.
   *
   * @param switchingDistanceInches New switching distance in inches
   * @return A new waypoint with updated switching distance
   */
  public ContinuousWaypoint withSwitchingDistanceInches(double switchingDistanceInches) {
    return withSwitchingDistance(Units.inchesToMeters(switchingDistanceInches));
  }

  @Override
  public String toString() {
    return String.format(
        "ContinuousWaypoint{pose=%s, switchingDist=%.2fm (%.1fin), final=%b}",
        targetPose,
        switchingDistanceMeters,
        Units.metersToInches(switchingDistanceMeters),
        isFinalWaypoint);
  }
}
