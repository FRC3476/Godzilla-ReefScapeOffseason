package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIOPhoton {
  @AutoLog
  public static class VisionIOPhotonInputs {
    public boolean connected = false;

    public Pose3d[] aprilTagPoses = new Pose3d[0];

    public Pose3d[] poses = new Pose3d[0];
    public Pose3d[] invalidPoses = new Pose3d[0];

    public PoseObservation[] poseObservations = new PoseObservation[0];
    public PoseObservation[] invalidPoseObservations = new PoseObservation[0];
  }

  public default void updateInputs(VisionIOPhotonInputs inputs) {}

  public default String getName() {
    return "";
  }

  public default Transform3d getRobotToCamera() {
    return new Transform3d();
  }
}
