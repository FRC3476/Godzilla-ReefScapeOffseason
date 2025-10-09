package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathfindToPoseCommand extends Command {
  private final PIDController xController =
      new PIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD);
  private final PIDController yController =
      new PIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD);
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveConstants.ANGLE_KP,
          0.0,
          DriveConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
  private final Supplier<Pose2d> targetPoseSupplier;
  private final Drive drive;
  private Command pathfindCommand;

  public PathfindToPoseCommand(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    xController.setTolerance(DriveConstants.AUTO_ALIGN_NORM_TOLERANCE);
    yController.setTolerance(DriveConstants.AUTO_ALIGN_NORM_TOLERANCE);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
    Logger.recordOutput("Odometry/Target Pathfinding Pose", FieldUtils.isRedAlliance());
    Pose2d pathfindingTarget = targetPoseSupplier.get();
    if (FieldUtils.isRedAlliance()) {
      pathfindingTarget = FlippingUtil.flipFieldPose(targetPoseSupplier.get());
    }
    pathfindCommand =
        AutoBuilder.pathfindToPoseFlipped(
            pathfindingTarget,
            new PathConstraints(
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                DriveConstants.MAX_TRANSLATIONAL_ACCEL,
                TunerConstants.kAngularSpeedAt12Volts.in(RadiansPerSecond),
                DriveConstants.MAX_ROTATIONAL_ACCEL),
            0.0);
    pathfindCommand.initialize();
  }

  @Override
  public void execute() {
    if (pathfindCommand.isFinished()) {
      Pose2d currentPose = drive.getPose();
      Pose2d targetPose = targetPoseSupplier.get();

      double xError = currentPose.getX() - targetPose.getX();
      double yError = currentPose.getY() - targetPose.getY();

      double xSpeed = xController.calculate(FieldUtils.getFlipped() * xError, 0.0);
      double ySpeed = yController.calculate(FieldUtils.getFlipped() * yError, 0.0);
      double omega =
          angleController.calculate(
              currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

      ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, omega);

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              FieldUtils.isRedAlliance()
                  ? drive.getRotation().plus(Rotation2d.k180deg)
                  : drive.getRotation()));
    } else {
      pathfindCommand.execute();
      angleController.reset(drive.getRotation().getRadians());
    }
  }

  @Override
  public void end(boolean interrupt) {
    xController.reset();
    yController.reset();
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
  }

  public Trigger atSetpoint() {
    return new Trigger(
        () -> xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint());
  }
}
