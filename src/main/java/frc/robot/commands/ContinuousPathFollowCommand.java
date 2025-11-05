package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ContinuousWaypoint;
import frc.robot.util.pathplanner.path.PathPlannerPath;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Command that drives the robot through multiple waypoints using Team 2056's continuous path
 * following approach.
 *
 * <p>Key features:
 *
 * <ul>
 *   <li>Dual-mode translation control: Max velocity for intermediate waypoints, PID for final
 *       waypoint
 *   <li>Distance-based waypoint switching with configurable thresholds (1.5" to 48")
 *   <li>Continuous rotation control throughout entire path
 * </ul>
 */
public class ContinuousPathFollowCommand extends Command {
  private final DriveSubsystem drive;
  private final List<ContinuousWaypoint> waypoints;
  private int currentWaypointIndex = 0;
  private boolean isAtFinalTarget = false;
  private final boolean resetPositionToStart;

  // Autopilot components for final waypoint PID control
  private final APConstraints constraints;
  private final APProfile profile;
  private final Autopilot autopilot;

  // Rotation PID controller (active throughout entire path)
  private final PIDController headingController =
      new PIDController(
          DriveConstants.AUTOPILOT_HEADING_KP,
          DriveConstants.AUTOPILOT_HEADING_KI,
          DriveConstants.AUTOPILOT_HEADING_KD);

  private final ApplyRobotSpeeds robotSpeedsRequest =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  /**
   * Creates a new ContinuousPathFollowCommand.
   *
   * @param drive The drive subsystem to control
   * @param waypoints List of waypoints to follow (must contain at least 2 waypoints, last one
   *     should be marked as final)
   * @param resetPositionToStart If true, resets robot odometry to the first waypoint pose on
   *     initialize
   */
  public ContinuousPathFollowCommand(
      DriveSubsystem drive, List<ContinuousWaypoint> waypoints, boolean resetPositionToStart) {
    this.drive = drive;
    this.waypoints = new ArrayList<>(waypoints);
    this.resetPositionToStart = resetPositionToStart;

    if (waypoints.size() < 2) {
      throw new IllegalArgumentException(
          "Continuous path must have at least 2 waypoints. Use DriveToPoseAutopilotCommand for single pose navigation.");
    }

    // Validate that only the last waypoint is marked as final
    for (int i = 0; i < waypoints.size() - 1; i++) {
      if (waypoints.get(i).isFinalWaypoint()) {
        throw new IllegalArgumentException(
            "Only the last waypoint should be marked as final. Waypoint "
                + i
                + " is incorrectly marked as final.");
      }
    }

    if (!waypoints.get(waypoints.size() - 1).isFinalWaypoint()) {
      throw new IllegalArgumentException(
          "The last waypoint must be marked as final. Use ContinuousWaypoint.finalWaypoint() for the last waypoint.");
    }

    // Initialize AutoPilot for final waypoint control
    this.constraints =
        new APConstraints()
            .withVelocity(DriveConstants.kDriveMaxSpeed)
            .withAcceleration(DriveConstants.AUTOPILOT_MAX_ACCELERATION)
            .withJerk(DriveConstants.AUTOPILOT_MAX_JERK);

    this.profile =
        new APProfile(constraints)
            .withErrorXY(Centimeters.of(DriveConstants.AUTOPILOT_ERROR_XY_METERS * 100))
            .withErrorTheta(Degrees.of(DriveConstants.AUTOPILOT_ERROR_THETA_DEGREES))
            .withBeelineRadius(
                Centimeters.of(DriveConstants.AUTOPILOT_BEELINE_RADIUS_METERS * 100));

    this.autopilot = new Autopilot(profile);

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(Math.toRadians(DriveConstants.AUTOPILOT_ERROR_THETA_DEGREES));

    addRequirements(drive);
  }

  /**
   * Creates a new ContinuousPathFollowCommand without resetting position.
   *
   * @param drive The drive subsystem to control
   * @param waypoints List of waypoints to follow (must contain at least 2 waypoints, last one
   *     should be marked as final)
   */
  public ContinuousPathFollowCommand(DriveSubsystem drive, List<ContinuousWaypoint> waypoints) {
    this(drive, waypoints, false);
  }

  /**
   * Creates a continuous path from a simple list of poses with default switching distances.
   *
   * @param drive The drive subsystem
   * @param poses List of poses (at least 2 required)
   * @param resetPositionToStart If true, resets robot odometry to the first pose
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPoses(
      DriveSubsystem drive, List<Pose2d> poses, boolean resetPositionToStart) {
    return fromPoses(
        drive,
        poses,
        DriveConstants.CONTINUOUS_PATH_DEFAULT_SWITCHING_DISTANCE_METERS,
        resetPositionToStart);
  }

  /**
   * Creates a continuous path from a simple list of poses with default switching distances without
   * resetting position.
   *
   * @param drive The drive subsystem
   * @param poses List of poses (at least 2 required)
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPoses(DriveSubsystem drive, List<Pose2d> poses) {
    return fromPoses(drive, poses, false);
  }

  /**
   * Creates a continuous path from a simple list of poses.
   *
   * @param drive The drive subsystem
   * @param poses List of poses (at least 2 required)
   * @param defaultSwitchingDistanceMeters Default switching distance for intermediate waypoints
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPoses(
      DriveSubsystem drive,
      List<Pose2d> poses,
      double defaultSwitchingDistanceMeters,
      boolean resetPositionToStart) {
    if (poses.size() < 2) {
      throw new IllegalArgumentException("Must provide at least 2 poses");
    }

    List<ContinuousWaypoint> waypoints = new ArrayList<>();
    for (int i = 0; i < poses.size() - 1; i++) {
      waypoints.add(new ContinuousWaypoint(poses.get(i), defaultSwitchingDistanceMeters, false));
    }
    // Add final waypoint
    waypoints.add(ContinuousWaypoint.finalWaypoint(poses.get(poses.size() - 1)));

    return new ContinuousPathFollowCommand(drive, waypoints, resetPositionToStart);
  }

  /**
   * Creates a continuous path from a simple list of poses without resetting position.
   *
   * @param drive The drive subsystem
   * @param poses List of poses (at least 2 required)
   * @param defaultSwitchingDistanceMeters Default switching distance for intermediate waypoints
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPoses(
      DriveSubsystem drive, List<Pose2d> poses, double defaultSwitchingDistanceMeters) {
    return fromPoses(drive, poses, defaultSwitchingDistanceMeters, false);
  }

  /**
   * Creates a continuous path from a variable number of poses.
   *
   * @param drive The drive subsystem
   * @param resetPositionToStart If true, resets robot odometry to the first pose
   * @param poses Array of poses (at least 2 required)
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPoses(
      DriveSubsystem drive, boolean resetPositionToStart, Pose2d... poses) {
    return fromPoses(drive, Arrays.asList(poses), resetPositionToStart);
  }

  /**
   * Creates a continuous path from a variable number of poses without resetting position.
   *
   * @param drive The drive subsystem
   * @param poses Array of poses (at least 2 required)
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPoses(DriveSubsystem drive, Pose2d... poses) {
    return fromPoses(drive, Arrays.asList(poses), false);
  }

  /**
   * Creates a continuous path from a PathPlanner path.
   *
   * <p>Extracts waypoints from the PathPlanner path and creates a continuous path. The path's
   * waypoints will be used with the default switching distance.
   *
   * @param drive The drive subsystem
   * @param pathName Name of the PathPlanner path to load
   * @param defaultSwitchingDistanceMeters Switching distance for intermediate waypoints
   * @param resetPositionToStart If true, resets robot odometry to the first waypoint pose
   * @param flipForAlliance If true, flips the path for red alliance
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive,
      String pathName,
      double defaultSwitchingDistanceMeters,
      boolean resetPositionToStart,
      boolean flipForAlliance) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return fromPathPlannerPath(
          drive, path, defaultSwitchingDistanceMeters, resetPositionToStart, flipForAlliance);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner path: " + pathName, e);
    }
  }

  /**
   * Creates a continuous path from a PathPlanner path with default alliance flipping enabled.
   *
   * @param drive The drive subsystem
   * @param pathName Name of the PathPlanner path to load
   * @param defaultSwitchingDistanceMeters Switching distance for intermediate waypoints
   * @param resetPositionToStart If true, resets robot odometry to the first waypoint pose
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive,
      String pathName,
      double defaultSwitchingDistanceMeters,
      boolean resetPositionToStart) {
    return fromPathPlannerPath(
        drive, pathName, defaultSwitchingDistanceMeters, resetPositionToStart, true);
  }

  /**
   * Creates a continuous path from a PathPlanner path without resetting position, with alliance
   * flipping enabled by default.
   *
   * @param drive The drive subsystem
   * @param pathName Name of the PathPlanner path to load
   * @param defaultSwitchingDistanceMeters Switching distance for intermediate waypoints
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive, String pathName, double defaultSwitchingDistanceMeters) {
    return fromPathPlannerPath(drive, pathName, defaultSwitchingDistanceMeters, false, true);
  }

  /**
   * Creates a continuous path from a PathPlanner path with default switching distance.
   *
   * @param drive The drive subsystem
   * @param pathName Name of the PathPlanner path to load
   * @param resetPositionToStart If true, resets robot odometry to the first waypoint pose
   * @param flipForAlliance If true, flips the path for red alliance
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive,
      String pathName,
      boolean resetPositionToStart,
      boolean flipForAlliance) {
    return fromPathPlannerPath(
        drive,
        pathName,
        DriveConstants.CONTINUOUS_PATH_DEFAULT_SWITCHING_DISTANCE_METERS,
        resetPositionToStart,
        flipForAlliance);
  }

  /**
   * Creates a continuous path from a PathPlanner path with default switching distance and alliance
   * flipping enabled by default.
   *
   * @param drive The drive subsystem
   * @param pathName Name of the PathPlanner path to load
   * @param resetPositionToStart If true, resets robot odometry to the first waypoint pose
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive, String pathName, boolean resetPositionToStart) {
    return fromPathPlannerPath(drive, pathName, resetPositionToStart, true);
  }

  /**
   * Creates a continuous path from a PathPlanner path with default switching distance without
   * resetting position, with alliance flipping enabled by default.
   *
   * @param drive The drive subsystem
   * @param pathName Name of the PathPlanner path to load
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive, String pathName) {
    return fromPathPlannerPath(drive, pathName, false, true);
  }

  /**
   * Creates a continuous path from an already loaded PathPlanner path.
   *
   * @param drive The drive subsystem
   * @param path The PathPlanner path
   * @param defaultSwitchingDistanceMeters Switching distance for intermediate waypoints
   * @param resetPositionToStart If true, resets robot odometry to the first waypoint pose
   * @param flipForAlliance If true, flips the path for red alliance
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive,
      PathPlannerPath path,
      double defaultSwitchingDistanceMeters,
      boolean resetPositionToStart,
      boolean flipForAlliance) {
    // Flip path if needed for red alliance
    boolean shouldFlip =
        flipForAlliance
            && DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    if (shouldFlip) {
      path = path.flipPath();
      Logger.recordOutput("ContinuousPathFollow/PathFlipped", true);
      Logger.recordOutput(
          "ContinuousPathFollow/Alliance", DriverStation.getAlliance().get().toString());
    } else {
      Logger.recordOutput("ContinuousPathFollow/PathFlipped", false);
      Logger.recordOutput(
          "ContinuousPathFollow/Alliance",
          DriverStation.getAlliance().isPresent()
              ? DriverStation.getAlliance().get().toString()
              : "None");
      Logger.recordOutput("ContinuousPathFollow/FlipForAllianceFlag", flipForAlliance);
    }
    // Extract waypoint poses from PathPlanner path
    List<Pose2d> poses = new ArrayList<>();

    // Get all bezier waypoints from the path and extract their anchor points
    var pathWaypoints = path.getWaypoints();
    var allPathPoints = path.getAllPathPoints();
    var rotationTargets = path.getRotationTargets();

    // For each waypoint, determine the appropriate rotation
    for (int wpIndex = 0; wpIndex < pathWaypoints.size(); wpIndex++) {
      var waypoint = pathWaypoints.get(wpIndex);
      Translation2d position = waypoint.anchor();
      Rotation2d rotation = Rotation2d.kZero;

      // Determine rotation based on waypoint position
      if (wpIndex == 0) {
        // First waypoint: use ideal starting state rotation if available
        if (path.getIdealStartingState() != null) {
          rotation = path.getIdealStartingState().rotation();
        } else {
          // Otherwise, use the initial heading direction
          rotation = path.getInitialHeading();
        }
      } else if (wpIndex == pathWaypoints.size() - 1) {
        // Last waypoint: use goal end state rotation
        if (path.getGoalEndState() != null) {
          rotation = path.getGoalEndState().rotation();
        } else {
          // Fallback to previous rotation
          rotation = poses.isEmpty() ? Rotation2d.kZero : poses.get(poses.size() - 1).getRotation();
        }
      } else {
        // Intermediate waypoints: check for rotation targets
        // Calculate waypoint relative position (0.0 to num_waypoints-1)
        double waypointRelativePosition = (double) wpIndex;

        // Find if there's a rotation target near this waypoint
        Rotation2d closestRotation = null;
        double closestDistance = Double.MAX_VALUE;

        for (var rotTarget : rotationTargets) {
          double distance = Math.abs(rotTarget.position() - waypointRelativePosition);
          if (distance < closestDistance && distance < 0.5) { // Within 0.5 waypoint units
            closestDistance = distance;
            closestRotation = rotTarget.rotation();
          }
        }

        if (closestRotation != null) {
          rotation = closestRotation;
        } else {
          // No rotation target found, calculate heading from path direction
          // Find the closest path point to this waypoint
          double minDist = Double.MAX_VALUE;
          int closestPointIndex = 0;
          for (int i = 0; i < allPathPoints.size(); i++) {
            double dist = allPathPoints.get(i).position.getDistance(position);
            if (dist < minDist) {
              minDist = dist;
              closestPointIndex = i;
            }
          }

          // Get rotation from the path point
          if (allPathPoints.get(closestPointIndex).rotationTarget != null) {
            rotation = allPathPoints.get(closestPointIndex).rotationTarget.rotation();
          } else if (closestPointIndex < allPathPoints.size() - 1) {
            // Calculate heading from path direction
            Translation2d direction =
                allPathPoints
                    .get(closestPointIndex + 1)
                    .position
                    .minus(allPathPoints.get(closestPointIndex).position);
            if (direction.getNorm() > 1e-6) {
              rotation = direction.getAngle();
            }
          }
        }
      }

      poses.add(new Pose2d(position, rotation));
    }

    // Create the command with the reset flag
    List<ContinuousWaypoint> waypoints = new ArrayList<>();
    for (int i = 0; i < poses.size() - 1; i++) {
      waypoints.add(new ContinuousWaypoint(poses.get(i), defaultSwitchingDistanceMeters, false));
    }
    waypoints.add(ContinuousWaypoint.finalWaypoint(poses.get(poses.size() - 1)));

    return new ContinuousPathFollowCommand(drive, waypoints, resetPositionToStart);
  }

  /**
   * Creates a continuous path from an already loaded PathPlanner path with alliance flipping
   * enabled by default.
   *
   * @param drive The drive subsystem
   * @param path The PathPlanner path
   * @param defaultSwitchingDistanceMeters Switching distance for intermediate waypoints
   * @param resetPositionToStart If true, resets robot odometry to the first waypoint pose
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive,
      PathPlannerPath path,
      double defaultSwitchingDistanceMeters,
      boolean resetPositionToStart) {
    return fromPathPlannerPath(
        drive, path, defaultSwitchingDistanceMeters, resetPositionToStart, true);
  }

  /**
   * Creates a continuous path from an already loaded PathPlanner path without resetting position,
   * with alliance flipping enabled by default.
   *
   * @param drive The drive subsystem
   * @param path The PathPlanner path
   * @param defaultSwitchingDistanceMeters Switching distance for intermediate waypoints
   * @return A new ContinuousPathFollowCommand
   */
  public static ContinuousPathFollowCommand fromPathPlannerPath(
      DriveSubsystem drive, PathPlannerPath path, double defaultSwitchingDistanceMeters) {
    return fromPathPlannerPath(drive, path, defaultSwitchingDistanceMeters, false, true);
  }

  @Override
  public void initialize() {
    currentWaypointIndex = 0;
    isAtFinalTarget = false;
    headingController.reset();

    // Reset robot position to first waypoint if flag is set
    if (resetPositionToStart && !waypoints.isEmpty()) {
      Pose2d startPose = waypoints.get(0).getTargetPose();
      drive.resetOdometry(startPose);
      Logger.recordOutput("Commands/" + getName() + "/ResetOdometryTo", startPose);
    }

    Logger.recordOutput("Commands/" + getName() + "/Active", true);
    Logger.recordOutput("Commands/" + getName() + "/TotalWaypoints", waypoints.size());
    Logger.recordOutput("Commands/" + getName() + "/ResetPositionToStart", resetPositionToStart);
  }

  @Override
  public void execute() {
    // Get current robot state
    Pose2d currentPose = RobotState.getGlobalPose();
    ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();

    // Get current waypoint
    ContinuousWaypoint currentWaypoint = waypoints.get(currentWaypointIndex);
    Pose2d targetPose = currentWaypoint.getTargetPose();

    // Calculate distance to current waypoint
    double distanceToWaypoint =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // Check if we should advance to next waypoint
    if (!currentWaypoint.isFinalWaypoint()
        && distanceToWaypoint < currentWaypoint.getSwitchingDistanceMeters()) {
      currentWaypointIndex++;
      currentWaypoint = waypoints.get(currentWaypointIndex);
      targetPose = currentWaypoint.getTargetPose();
      distanceToWaypoint = currentPose.getTranslation().getDistance(targetPose.getTranslation());

      Logger.recordOutput(
          "Commands/" + getName() + "/WaypointAdvanced",
          currentWaypointIndex + " / " + waypoints.size());
    }

    // Calculate translational velocity based on waypoint type
    double vxFieldRelative;
    double vyFieldRelative;

    if (currentWaypoint.isFinalWaypoint()) {
      // FINAL WAYPOINT: Use Autopilot PID for precise stopping
      APTarget target = new APTarget(targetPose);
      APResult result = autopilot.calculate(currentPose, robotRelativeSpeeds, target);

      vxFieldRelative = result.vx().in(MetersPerSecond);
      vyFieldRelative = result.vy().in(MetersPerSecond);

      // Check if we're at the final target
      isAtFinalTarget = autopilot.atTarget(currentPose, target);
    } else {
      // INTERMEDIATE WAYPOINT: Continuous move - command max velocity
      // Calculate direction to target
      double angleToTarget =
          Math.atan2(
              targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());

      // Command full velocity in the direction of the target
      double velocity = DriveConstants.CONTINUOUS_PATH_INTERMEDIATE_VELOCITY;
      vxFieldRelative = velocity * Math.cos(angleToTarget);
      vyFieldRelative = velocity * Math.sin(angleToTarget);

      isAtFinalTarget = false;
    }

    // Calculate heading control (ALWAYS ACTIVE for all waypoints)
    double headingVelocity =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Convert field-relative velocities to robot-relative
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vxFieldRelative, vyFieldRelative, headingVelocity, currentPose.getRotation());

    // Apply robot-relative speeds
    drive.setControl(robotSpeedsRequest.withSpeeds(speeds));

    // Comprehensive logging
    Logger.recordOutput("Commands/" + getName() + "/CurrentWaypointIndex", currentWaypointIndex);
    Logger.recordOutput("Commands/" + getName() + "/CurrentPose", currentPose);
    Logger.recordOutput("Commands/" + getName() + "/TargetPose", targetPose);
    Logger.recordOutput("Commands/" + getName() + "/DistanceToWaypoint", distanceToWaypoint);
    Logger.recordOutput(
        "Commands/" + getName() + "/SwitchingDistance",
        currentWaypoint.getSwitchingDistanceMeters());
    Logger.recordOutput(
        "Commands/" + getName() + "/IsFinalWaypoint", currentWaypoint.isFinalWaypoint());
    Logger.recordOutput("Commands/" + getName() + "/FieldRelVelX", vxFieldRelative);
    Logger.recordOutput("Commands/" + getName() + "/FieldRelVelY", vyFieldRelative);
    Logger.recordOutput("Commands/" + getName() + "/HeadingVelocity", headingVelocity);
    Logger.recordOutput("Commands/" + getName() + "/AppliedSpeeds", speeds);
    Logger.recordOutput("Commands/" + getName() + "/AtFinalTarget", isAtFinalTarget);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drive.setControl(robotSpeedsRequest.withSpeeds(new ChassisSpeeds()));

    Logger.recordOutput("Commands/" + getName() + "/Active", false);
    Logger.recordOutput("Commands/" + getName() + "/Interrupted", interrupted);
    Logger.recordOutput(
        "Commands/" + getName() + "/CompletedWaypoints",
        currentWaypointIndex + " / " + waypoints.size());
  }

  @Override
  public boolean isFinished() {
    // Command finishes when we reach the final target
    return isAtFinalTarget;
  }

  /**
   * Returns a trigger that is true when the robot is at the final target pose.
   *
   * @return A trigger that activates when the robot reaches the final target
   */
  public Trigger atFinalTarget() {
    return new Trigger(() -> isAtFinalTarget);
  }

  /**
   * Returns a trigger that is true when the robot reaches a specific waypoint index.
   *
   * @param waypointIndex The waypoint index to check
   * @return A trigger that activates when reaching the specified waypoint
   */
  public Trigger atWaypoint(int waypointIndex) {
    return new Trigger(() -> currentWaypointIndex >= waypointIndex);
  }

  /**
   * Gets the current waypoint index.
   *
   * @return The index of the current waypoint being targeted
   */
  public int getCurrentWaypointIndex() {
    return currentWaypointIndex;
  }

  /**
   * Gets the total number of waypoints in the path.
   *
   * @return Total number of waypoints
   */
  public int getTotalWaypoints() {
    return waypoints.size();
  }
}
