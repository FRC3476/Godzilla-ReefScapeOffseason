package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotState;
import java.util.List;

public class GenerateAndFollowPathCommand extends Command {
  private final Pose2d targetPose;
  private final RobotState robotState;
  private Command followPathCommand;

  public GenerateAndFollowPathCommand(RobotState robotState, Pose2d targetPose) {
    this.targetPose =
        FieldUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(targetPose) : targetPose;
    this.robotState = robotState;
  }

  @Override
  public void initialize() {
    ChassisSpeeds speeds = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
    // direction is direction of travel, not target direction
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                RobotState.getGlobalPose().getTranslation(),
                new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)),
            targetPose);

    PathConstraints constraints =
        new PathConstraints(
            2,
            2,
            Units.degreesToRadians(540.000),
            Units.degreesToRadians(720.000)); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(0.0, targetPose.getRotation()));
    path.preventFlipping = true;
    followPathCommand = AutoBuilder.followPath(path);
    followPathCommand.initialize();
  }

  @Override
  public void execute() {
    followPathCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return followPathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupt) {
    followPathCommand.end(interrupt);
  }
}
