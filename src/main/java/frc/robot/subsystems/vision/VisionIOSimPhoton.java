package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Constants.VisionConstants;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class VisionIOSimPhoton extends VisionIOHardwarePhoton {
  private final PhotonCameraSim cameraSim;

  public VisionIOSimPhoton(String cameraName, Transform3d robotToCamera) {
    super(cameraName, robotToCamera);

    SimCameraProperties cameraProp = new SimCameraProperties();

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProp);

    cameraSim.enableDrawWireframe(true);
    cameraSim.setWireframeResolution(1);
    cameraSim.setMaxSightRange(10);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);

    VisionConstants.visionSim.ifPresent(visionSim -> visionSim.addCamera(cameraSim, robotToCamera));
  }

  @Override
  public void updateInputs(VisionIOPhotonInputs inputs) {
    super.updateInputs(inputs);

    VisionConstants.visionSim.ifPresent(
        visionSim -> {
          FieldObject2d visionEstimation = visionSim.getDebugField().getObject("VisionEstimation");

          if (inputs.poseObservations.length != 0) {
            visionEstimation.setPoses(inputs.poseObservations[0].robotPose().toPose2d());
          } else {
            visionEstimation.setPoses();
          }
        });
  }
}
