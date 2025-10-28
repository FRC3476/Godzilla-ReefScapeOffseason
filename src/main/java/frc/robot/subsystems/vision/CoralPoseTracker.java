package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Optional;
import java.util.Set;

public class CoralPoseTracker {
  private Set<CoralPoseObservation> observations;

  public class CoralPoseObservation implements Struct<CoralPoseObservation> {
    Translation2d pose;
    double lastObservation;
    int numObservations;

    CoralPoseObservation(Pose2d pose) {
      this.pose = pose.getTranslation();
      this.lastObservation = Timer.getFPGATimestamp();
      this.numObservations = 1;
    }

    CoralPoseObservation(double posex, double posey, double lastObservation, int numObservations) {
      this.pose = new Translation2d(posex, posey);
      this.lastObservation = lastObservation;
      this.numObservations = numObservations;
    }

    CoralPoseObservation() {
      this.pose = Translation2d.kZero;
      this.lastObservation = 0;
      this.numObservations = 0;
    }

    @Override
    public Class<CoralPoseObservation> getTypeClass() {
      return CoralPoseObservation.class;
    }

    @Override
    public String getTypeName() {
      return "CoralPoseObservation";
    }

    @Override
    public int getSize() {
      return 16 + 8 + 4; // translation2d 16 bytes, double 8 bytes, int 4 bytes
    }

    @Override
    public String getSchema() {
      return "Pose2d pose; double lastObservation; int numObservations";
    }

    @Override
    public void pack(ByteBuffer bb, CoralPoseObservation data) {
      bb.putDouble(pose.getX());
      bb.putDouble(pose.getY());
      bb.putDouble(lastObservation);
      bb.putInt(numObservations);
    }

    @Override
    public CoralPoseObservation unpack(ByteBuffer bb) {
      double posex = bb.getDouble();
      double posey = bb.getDouble();
      double lastObservation = bb.getDouble();
      int numObservations = bb.getInt();
      return new CoralPoseObservation(posex, posey, lastObservation, numObservations);
    }

    boolean isClose(Pose2d other) {
      return other.getTranslation().minus(pose).getNorm()
          < VisionConstants.coralObservationDistanceThreshold;
    }

    void update(Pose2d newPose) {
      pose = newPose.getTranslation();
      lastObservation = Timer.getFPGATimestamp();
      numObservations++;
    }

    double distanceToRobot() {
      return RobotState.getGlobalPose().getTranslation().minus(pose).getNorm();
    }
  }

  public CoralPoseTracker() {
    this.observations = new HashSet<>();
  }

  public CoralPoseObservation[] getObservations() {
    return observations.toArray(new CoralPoseObservation[0]);
  }

  public void addObservations(ArrayList<Pose2d> coralPoses) {
    for (Pose2d coralPose : coralPoses) {
      boolean newObservation = true;
      for (CoralPoseObservation observation : observations) {
        if (observation.isClose(coralPose)) {
          observation.update(coralPose);
          newObservation = false;
          break;
        }
      }
      if (newObservation) {
        observations.add(new CoralPoseObservation(coralPose));
      }
    }
    Iterator<CoralPoseObservation> iterator = observations.iterator();
    while (iterator.hasNext()) {
      CoralPoseObservation observation = iterator.next();
      if (Timer.getFPGATimestamp() - observation.lastObservation
          > VisionConstants.coralObservationTimeThreshold) {
        iterator.remove();
      }
    }
  }

  public Optional<Pose2d> getCoralPose() {
    if (observations.isEmpty()) {
      return Optional.empty();
    }
    CoralPoseObservation bestObservation = new CoralPoseObservation();
    for (CoralPoseObservation observation : observations) {
      if (observation.distanceToRobot() < bestObservation.distanceToRobot()
          || bestObservation.numObservations < VisionConstants.coralObservationMinObservations
              && observation.numObservations >= VisionConstants.coralObservationMinObservations) {
        bestObservation = observation;
      }
    }
    if (bestObservation.numObservations < VisionConstants.coralObservationMinObservations) {
      return Optional.empty();
    }
    return Optional.of(new Pose2d(bestObservation.pose, Rotation2d.kZero));
  }
}
