package frc.robot.Field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Field.FieldConstants.AprilTagStruct;
import frc.robot.util.MathHelpers;

public record ReefPole(AprilTagStruct tag, double tagToReef) {
  public Pose2d getPose() {
    return MathHelpers.getParallelOffsetPose(tag.pose().toPose2d(), tagToReef);
  }

  public double getParallellError(Pose2d robotPose) {
    return MathHelpers.getParallelError(robotPose, tag.pose().toPose2d());
  }
}
