package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Optional;

/** Interface for vision system hardware abstraction. */
public interface VisionIOLimelight {

  /** Container for all vision input data. */
  class VisionIOInputs {
    /** Input data from a single camera. */
    public static class CameraInputs {
      public boolean seesTarget;
      public FiducialObservation[] fiducialObservations;
      public MegatagPoseEstimate megatagPoseEstimate;
      public MegatagPoseEstimate megatag2PoseEstimate;
      public int megatag2Count;
      public int megatagCount;
      public double megatagDistance;
      public double megatag2Distance;
      public Pose3d pose3d;
      public double[] standardDeviations =
          new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
      // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    }

    public CameraInputs cameraA = new CameraInputs();
    public CameraInputs cameraB = new CameraInputs();
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default boolean isCoralDetected() {
    return false;
  }

  default double getCoralTx() {
    return 0.0;
  }

  default double getCoralTy() {
    return 0.0;
  }

  default double getCoralTxNc() {
    return 0.0;
  }

  default double getCoralTyNc() {
    return 0.0;
  }

  default Optional<ArrayList<Pair<Double, Double>>> getAllCoralTNCs() {
    return Optional.of(new ArrayList<Pair<Double, Double>>());
  }
}
