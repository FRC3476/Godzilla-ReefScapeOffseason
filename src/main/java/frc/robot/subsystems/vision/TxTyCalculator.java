package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

/** Utility class for calculating tx/ty angles from fiducial observations. */
public class TxTyCalculator {

  /**
   * Calculates average horizontal and vertical angles from a fiducial observation.
   *
   * <p>Limelights provide normalized coordinates (txnc, tync) ranging from -1 to 1. This method
   * converts them to angles in radians using the camera's field of view.
   *
   * @param observation The fiducial observation containing normalized coordinates
   * @return Array with [tx, ty] in radians, or null if observation is null
   */
  public static double[] calculateAngles(FiducialObservation observation) {
    if (observation == null) {
      return null;
    }

    // Limelight provides normalized coordinates from -1 to 1
    // Convert to angles using half the FOV (since normalized coords go from -1 to 1)
    double tx =
        observation.txnc()
            * Units.degreesToRadians(VisionConstants.kLimelight3HorizontalFOVDegrees / 2.0);
    double ty =
        observation.tync()
            * Units.degreesToRadians(VisionConstants.kLimelight3VerticalFOVDegrees / 2.0);

    return new double[] {tx, ty};
  }

  /**
   * Calculates average horizontal and vertical angles from multiple fiducial observations (e.g.,
   * corners of a tag).
   *
   * @param observations Array of fiducial observations
   * @return Array with [average_tx, average_ty] in radians, or null if no valid observations
   */
  public static double[] calculateAverageAngles(FiducialObservation[] observations) {
    if (observations == null || observations.length == 0) {
      return null;
    }

    double sumTx = 0.0;
    double sumTy = 0.0;
    int count = 0;

    for (FiducialObservation obs : observations) {
      if (obs != null) {
        double[] angles = calculateAngles(obs);
        if (angles != null) {
          sumTx += angles[0];
          sumTy += angles[1];
          count++;
        }
      }
    }

    if (count == 0) {
      return null;
    }

    return new double[] {sumTx / count, sumTy / count};
  }
}

