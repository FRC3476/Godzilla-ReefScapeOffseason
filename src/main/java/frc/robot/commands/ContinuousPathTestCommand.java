package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ContinuousWaypoint;
import java.util.ArrayList;
import java.util.List;

/**
 * Test command for continuous path following.
 *
 * <p>Creates a simple test path with multiple waypoints to demonstrate Team 2056's continuous move
 * approach. The robot will smoothly navigate through intermediate waypoints without stopping, then
 * precisely stop at the final waypoint.
 */
public class ContinuousPathTestCommand {

  /**
   * Creates a simple square path test that demonstrates continuous path following.
   *
   * <p>The robot will drive in a square pattern with 4 waypoints, maintaining velocity through the
   * first 3 corners and stopping at the final corner.
   *
   * @param container The robot container
   * @return A command that executes the square test pattern
   */
  public static Command squareTest(RobotContainer container) {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting Continuous Path Square Test")),
        createSquarePath(container, 2.0), // 2 meter square
        Commands.runOnce(() -> System.out.println("Completed Continuous Path Square Test")));
  }

  /**
   * Creates a zigzag path test that demonstrates aggressive direction changes.
   *
   * <p>The robot will drive in a zigzag pattern, demonstrating how it handles continuous moves with
   * larger angle changes between waypoints.
   *
   * @param container The robot container
   * @return A command that executes the zigzag test pattern
   */
  public static Command zigzagTest(RobotContainer container) {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting Continuous Path Zigzag Test")),
        createZigzagPath(container),
        Commands.runOnce(() -> System.out.println("Completed Continuous Path Zigzag Test")));
  }

  /**
   * Creates a test path that demonstrates different switching distances.
   *
   * <p>Each intermediate waypoint uses a different switching distance to show how this affects the
   * path smoothness and when waypoint transitions occur.
   *
   * @param container The robot container
   * @return A command that executes the variable switching distance test
   */
  public static Command variableSwitchingDistanceTest(RobotContainer container) {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting Variable Switching Distance Test")),
        createVariableSwitchingPath(container),
        Commands.runOnce(() -> System.out.println("Completed Variable Switching Distance Test")));
  }

  /**
   * Creates a simple forward path (single line) with intermediate waypoints.
   *
   * @param container The robot container
   * @return A command that drives forward through multiple waypoints
   */
  public static Command forwardPathTest(RobotContainer container) {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting Forward Path Test")),
        createForwardPath(container, 4.0), // 4 meters forward with waypoints
        Commands.runOnce(() -> System.out.println("Completed Forward Path Test")));
  }

  // ==================== Path Creation Methods ====================

  private static Command createSquarePath(RobotContainer container, double sideLength) {
    DriveSubsystem drive = container.getDrive();
    Pose2d startPose = RobotState.getGlobalPose();

    List<ContinuousWaypoint> waypoints = new ArrayList<>();

    // Start at current position
    waypoints.add(ContinuousWaypoint.intermediate(startPose));

    // Forward (positive X)
    waypoints.add(
        ContinuousWaypoint.intermediate(
            new Pose2d(
                startPose.getX() + sideLength,
                startPose.getY(),
                startPose.getRotation().rotateBy(Rotation2d.kCCW_90deg))));

    // Left (positive Y)
    waypoints.add(
        ContinuousWaypoint.intermediate(
            new Pose2d(
                startPose.getX() + sideLength,
                startPose.getY() + sideLength,
                startPose.getRotation().rotateBy(Rotation2d.k180deg))));

    // Backward (negative X)
    waypoints.add(
        ContinuousWaypoint.intermediate(
            new Pose2d(
                startPose.getX(),
                startPose.getY() + sideLength,
                startPose.getRotation().rotateBy(Rotation2d.kCW_90deg))));

    // Return to start (negative Y) - FINAL WAYPOINT
    waypoints.add(ContinuousWaypoint.finalWaypoint(startPose));

    return new ContinuousPathFollowCommand(drive, waypoints);
  }

  private static Command createZigzagPath(RobotContainer container) {
    DriveSubsystem drive = container.getDrive();
    Pose2d startPose = RobotState.getGlobalPose();
    double zigzagDistance = 1.5; // 1.5 meters per leg
    double zigzagWidth = 1.0; // 1 meter offset

    List<ContinuousWaypoint> waypoints = new ArrayList<>();

    // Start
    waypoints.add(ContinuousWaypoint.intermediate(startPose));

    // Zigzag pattern: 5 waypoints creating a zigzag
    for (int i = 1; i <= 4; i++) {
      double x = startPose.getX() + (zigzagDistance * i);
      double y = startPose.getY() + (i % 2 == 0 ? 0 : zigzagWidth);
      Rotation2d rotation = (i % 2 == 0) ? Rotation2d.fromDegrees(-45) : Rotation2d.fromDegrees(45);

      if (i < 4) {
        waypoints.add(
            ContinuousWaypoint.intermediateInches(
                new Pose2d(x, y, rotation), 18.0)); // 18" switching distance
      } else {
        // Final waypoint
        waypoints.add(ContinuousWaypoint.finalWaypoint(new Pose2d(x, y, rotation)));
      }
    }

    return new ContinuousPathFollowCommand(drive, waypoints);
  }

  private static Command createVariableSwitchingPath(RobotContainer container) {
    DriveSubsystem drive = container.getDrive();
    Pose2d startPose = RobotState.getGlobalPose();

    List<ContinuousWaypoint> waypoints = new ArrayList<>();

    // Start
    waypoints.add(
        ContinuousWaypoint.intermediate(
            startPose, Units.inchesToMeters(6.0))); // Small switching distance (6")

    // Waypoint 1 - Medium switching distance
    waypoints.add(
        ContinuousWaypoint.intermediate(
            new Pose2d(startPose.getX() + 1.5, startPose.getY(), Rotation2d.kZero),
            Units.inchesToMeters(12.0))); // 12" switching distance

    // Waypoint 2 - Large switching distance
    waypoints.add(
        ContinuousWaypoint.intermediate(
            new Pose2d(startPose.getX() + 3.0, startPose.getY() + 1.0, Rotation2d.kCCW_90deg),
            Units.inchesToMeters(24.0))); // 24" switching distance

    // Waypoint 3 - Very large switching distance (like Team 2056's max)
    waypoints.add(
        ContinuousWaypoint.intermediate(
            new Pose2d(startPose.getX() + 4.0, startPose.getY() + 2.0, Rotation2d.k180deg),
            Units.inchesToMeters(48.0))); // 48" switching distance

    // Final waypoint
    waypoints.add(
        ContinuousWaypoint.finalWaypoint(
            new Pose2d(startPose.getX() + 5.0, startPose.getY() + 2.0, Rotation2d.k180deg)));

    return new ContinuousPathFollowCommand(drive, waypoints);
  }

  private static Command createForwardPath(RobotContainer container, double totalDistance) {
    DriveSubsystem drive = container.getDrive();
    Pose2d startPose = RobotState.getGlobalPose();
    int numWaypoints = 5; // 5 waypoints including start and end
    double distancePerWaypoint = totalDistance / (numWaypoints - 1);

    List<ContinuousWaypoint> waypoints = new ArrayList<>();

    for (int i = 0; i < numWaypoints; i++) {
      double x = startPose.getX() + (distancePerWaypoint * i);
      Pose2d pose = new Pose2d(x, startPose.getY(), startPose.getRotation());

      if (i < numWaypoints - 1) {
        // Intermediate waypoints - use default switching distance
        waypoints.add(ContinuousWaypoint.intermediate(pose));
      } else {
        // Final waypoint
        waypoints.add(ContinuousWaypoint.finalWaypoint(pose));
      }
    }

    return new ContinuousPathFollowCommand(drive, waypoints);
  }

  /**
   * Creates a comprehensive test that runs multiple path tests in sequence.
   *
   * <p>This is useful for thorough validation of the continuous path following system.
   *
   * @param container The robot container
   * @return A command that runs all test patterns
   */
  public static Command comprehensiveTest(RobotContainer container) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                System.out.println(
                    "========================================\n"
                        + "Starting Comprehensive Continuous Path Test\n"
                        + "Testing Team 2056's continuous move approach\n"
                        + "========================================")),
        Commands.waitSeconds(1.0),
        // Test 1: Simple forward path
        Commands.runOnce(() -> System.out.println("\n--- Test 1: Forward Path ---")),
        forwardPathTest(container),
        Commands.waitSeconds(2.0),
        // Test 2: Variable switching distances
        Commands.runOnce(
            () -> System.out.println("\n--- Test 2: Variable Switching Distances ---")),
        variableSwitchingDistanceTest(container),
        Commands.waitSeconds(2.0),
        Commands.runOnce(
            () ->
                System.out.println(
                    "\n========================================\n"
                        + "Comprehensive Test Complete!\n"
                        + "========================================")));
  }

  /**
   * Creates a test command that follows RightRaw1.path and RightRaw2.path in sequence using
   * continuous path following.
   *
   * <p>This demonstrates using real PathPlanner paths with the continuous move approach. The robot
   * position will be reset to the start of RightRaw1 path to ensure accurate path following. The
   * paths will automatically flip for red alliance (default behavior).
   *
   * @param container The robot container
   * @return A command that follows RightRaw1 then RightRaw2 paths
   */
  public static Command rightRawPathTest(RobotContainer container) {
    DriveSubsystem drive = container.getDrive();
    return Commands.sequence(
        Commands.runOnce(
            () -> System.out.println("Starting Right Raw Path Test (RightRaw1 -> RightRaw2)")),
        // Follow RightRaw1.path with position reset (alliance flipping is default)
        ContinuousPathFollowCommand.fromPathPlannerPath(drive, "RightRaw1", true),
        Commands.waitSeconds(0.5), // Brief pause between paths
        // Follow RightRaw2.path (no reset, continue from current position)
        ContinuousPathFollowCommand.fromPathPlannerPath(drive, "RightRaw2", false),
        Commands.runOnce(() -> System.out.println("Completed Right Raw Path Test")));
  }

  /**
   * Creates a test command that demonstrates the difference between continuous path following and
   * standard point-to-point navigation.
   *
   * <p>Runs the same path twice: once with continuous following, once with standard
   * DriveToPoseAutopilotCommand for each waypoint.
   *
   * @param container The robot container
   * @return A comparison test command
   */
  public static Command comparisonTest(RobotContainer container) {
    DriveSubsystem drive = container.getDrive();
    Pose2d startPose = RobotState.getGlobalPose();

    // Define waypoints for the test
    List<Pose2d> testPoses =
        List.of(
            startPose,
            new Pose2d(startPose.getX() + 2.0, startPose.getY(), Rotation2d.kZero),
            new Pose2d(startPose.getX() + 2.0, startPose.getY() + 2.0, Rotation2d.kCCW_90deg),
            new Pose2d(startPose.getX(), startPose.getY() + 2.0, Rotation2d.k180deg),
            startPose);

    return Commands.sequence(
        Commands.runOnce(
            () ->
                System.out.println(
                    "========================================\n"
                        + "Comparison Test: Continuous vs Point-to-Point\n"
                        + "========================================")),
        // Test with continuous path following
        Commands.runOnce(
            () -> System.out.println("\n--- Running CONTINUOUS PATH (Team 2056 approach) ---")),
        ContinuousPathFollowCommand.fromPoses(
            drive, testPoses, DriveConstants.CONTINUOUS_PATH_DEFAULT_SWITCHING_DISTANCE_METERS),
        Commands.waitSeconds(3.0),
        // Test with standard point-to-point
        Commands.runOnce(
            () -> System.out.println("\n--- Running POINT-TO-POINT (Standard AutoPilot) ---")),
        Commands.sequence(
            testPoses.stream()
                .map(pose -> new DriveToPoseAutopilotCommand(container, pose))
                .toArray(Command[]::new)),
        Commands.runOnce(
            () ->
                System.out.println(
                    "\n========================================\n"
                        + "Comparison Test Complete!\n"
                        + "Notice the difference in smoothness and timing.\n"
                        + "========================================")));
  }
}
