package frc.robot.subsystems.vision;

/**
 * Represents a TxTy observation for trig-based local pose estimation.
 *
 * <p>This observation combines horizontal/vertical angles to a single AprilTag with the
 * unambiguous 3D distance from SolvePNP to enable stable local pose estimation for precise
 * alignment.
 *
 * @param tagId The AprilTag fiducial ID
 * @param cameraIndex Which camera captured the observation (0 = A/left, 1 = B/right)
 * @param tx Average horizontal angle to tag in radians
 * @param ty Average vertical angle to tag in radians
 * @param distance3d 3D distance from camera to tag in meters (from SolvePNP)
 * @param timestampSeconds When the observation was captured (FPGA timestamp)
 */
public record TxTyObservation(
    int tagId, int cameraIndex, double tx, double ty, double distance3d, double timestampSeconds) {}

