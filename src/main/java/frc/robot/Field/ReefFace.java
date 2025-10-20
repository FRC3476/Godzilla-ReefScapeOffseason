package frc.robot.Field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public class ReefFace {
  public final AprilTagStruct tag;
  public final ReefPole leftPole;
  public final ReefPole rightPole;

  public ReefFace(AprilTagStruct tag) {
    this.tag = tag;
    this.leftPole = new ReefPole(tag, -FieldConstants.tagToReef);
    this.rightPole = new ReefPole(tag, FieldConstants.tagToReef);
  }
  
  public Pose2d getPose() {
    return tag.pose().toPose2d();
  }

  public double getPerpendicularError(Pose2d robotPose) {
    return PoseUtils.getPerpendicularError(robotPose, tag.pose().toPose2d());
  }
}
