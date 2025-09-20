package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final VisionIO io;

  private static Vision visionSubsystem;

  public static Vision getInstance() {
    if (visionSubsystem == null) {
      visionSubsystem = new Vision(new VisionIOHardwareLimelight(null));
    }
    return visionSubsystem;
  }

  public Vision(VisionIO io) {
    this.io = io;
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
}
