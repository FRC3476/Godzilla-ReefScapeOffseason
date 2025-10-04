package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  private static Vision visionSubsystem;

  public static Vision getInstance() {
    if (visionSubsystem == null) {
      visionSubsystem = new Vision(new VisionIOHardwareLimelight());
    }
    return visionSubsystem;
  }

  public Vision(VisionIO io) {
    this.io = io;
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Vision/CameraA/SeesTarget", inputs.cameraA.seesTarget);
    Logger.recordOutput("Vision/CameraB/SeesTarget", inputs.cameraB.seesTarget);
    Logger.recordOutput("Vision/CameraA/Pose", inputs.cameraA.pose3d);
    Logger.recordOutput("Vision/CameraB/Pose", inputs.cameraB.pose3d);
  }

  // object detection methods

  public boolean isCoralDetected() {
    return io.isCoralDetected();
  }

  public double getCoralTx() {
    return io.getCoralTx();
  }

  public double getCoralTy() {
    return io.getCoralTy();
  }

  public VisionIO.VisionIOInputs.CameraInputs getCameraAInputs() {
    return inputs.cameraA;
  }

  public VisionIO.VisionIOInputs.CameraInputs getCameraBInputs() {
    return inputs.cameraB;
  }
}
