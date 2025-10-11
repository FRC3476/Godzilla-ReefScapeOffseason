package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.util.MathHelpers;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code DriveIO} interface defines the input/output operations for the drivetrain. It provides
 * methods to control and monitor the swerve drive system.
 */
public interface DriveIO {

  @AutoLog
  class DriveIOInputs extends SwerveDriveState {
    public double gyroAngle = 0.0;

    DriveIOInputs() {
      this.Pose = MathHelpers.kPose2dZero;
    }

    public void fromSwerveDriveState(SwerveDriveState stateIn) {
      this.Pose = stateIn.Pose;
      this.SuccessfulDaqs = stateIn.SuccessfulDaqs;
      this.FailedDaqs = stateIn.FailedDaqs;
      this.ModuleStates = stateIn.ModuleStates;
      this.ModuleTargets = stateIn.ModuleTargets;
      this.Speeds = stateIn.Speeds;
      this.OdometryPeriod = stateIn.OdometryPeriod;
    }
  }

  default void readInputs(DriveIOInputs inputs) {}

  default void logModules(SwerveDriveState driveState) {}

  default void resetOdometry(Pose2d pose) {}

  default void setControl(SwerveRequest request) {}

  default Command applyRequest(
      Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
    return Commands.none();
  }

  default void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {}

  default void updateOperatorPerspective() {}

  default void setStateStdDevs(double xStd, double yStd, double rotStd) {}
}
