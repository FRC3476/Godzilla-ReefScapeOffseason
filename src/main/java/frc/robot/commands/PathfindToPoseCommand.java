package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.pathplanner.auto.AutoBuilder;
import frc.robot.util.pathplanner.path.PathConstraints;
import frc.robot.util.pathplanner.util.FlippingUtil;
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
          new TrapezoidProfile.Constraints(8.0, DriveConstants.ANGLE_MAX_ACCELERATION));
  private final Supplier<Pose2d> targetPoseSupplier;
  private final DriveSubsystem drive;
  private Command pathfindCommand;

  private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  static final double kPathfindSpeedScalingFactor = 0.8;
  static final double kPathfindAccelScalingFactor = 0.8;
  static final double kPathfindYAccelScalingFactor = 0.4;
  static final double kPathfindAngularScalingFactor = 0.8;

  public PathfindToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    xController.setTolerance(DriveConstants.AUTO_ALIGN_NORM_TOLERANCE);
    yController.setTolerance(DriveConstants.AUTO_ALIGN_NORM_TOLERANCE);
    angleController.setTolerance(Units.degreesToRadians(1.5));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    angleController.reset(RobotState.getGlobalPose().getRotation().getRadians());
    Logger.recordOutput("Odometry/Target Pathfinding Pose", FieldUtils.isRedAlliance());
    Pose2d pathfindingTarget = targetPoseSupplier.get();
    if (FieldUtils.isRedAlliance()) {
      pathfindingTarget = FlippingUtil.flipFieldPose(targetPoseSupplier.get());
    }
    pathfindCommand =
        AutoBuilder.pathfindToPoseFlipped(
            pathfindingTarget,
            new PathConstraints(
                Constants.DriveConstants.kDriveMaxSpeed * kPathfindSpeedScalingFactor,
                Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared
                    * kPathfindAccelScalingFactor,
                Constants.DriveConstants.kDriveMaxAngularRate * kPathfindAngularScalingFactor,
                Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                    * kPathfindAngularScalingFactor,
                Constants.DriveConstants.kMaxXAccelerationMetersPerSecondSquared
                    * kPathfindAccelScalingFactor,
                Constants.DriveConstants.kMaxYAccelerationMetersPerSecondSquared
                    * kPathfindYAccelScalingFactor),
            0.0);
    pathfindCommand.initialize();
  }

  @Override
  public void execute() {
    if (pathfindCommand.isFinished()) {
      Pose2d currentPose = RobotState.getGlobalPose();
      Pose2d targetPose = targetPoseSupplier.get();

      double xError = currentPose.getX() - targetPose.getX();
      Logger.recordOutput("Commands/" + getName() + "/xError", xError);
      double yError = currentPose.getY() - targetPose.getY();
      Logger.recordOutput("Commands/" + getName() + "/yError", yError);

      double xSpeed = xController.calculate(FieldUtils.getFlipped() * xError, 0.0);
      double ySpeed = yController.calculate(FieldUtils.getFlipped() * yError, 0.0);
      double omega =
          angleController.calculate(
              currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

      ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, omega);

      drive.applyRequest(
          () ->
              robotSpeeds.withSpeeds(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      FieldUtils.isRedAlliance()
                          ? RobotState.getGlobalPose().getRotation().plus(Rotation2d.k180deg)
                          : RobotState.getGlobalPose().getRotation())));

      Logger.recordOutput(
          "Commands/" + getName() + "/xControllerAtSetpoint", xController.atSetpoint());
      Logger.recordOutput(
          "Commands/" + getName() + "/yControllerAtSetpoint", yController.atSetpoint());
      Logger.recordOutput(
          "Commands/" + getName() + "/AngleAtSetpoint", angleController.atSetpoint());
    } else {
      pathfindCommand.execute();
      angleController.reset(RobotState.getGlobalPose().getRotation().getRadians());
    }
  }

  @Override
  public void end(boolean interrupt) {
    pathfindCommand.end(true);
    xController.reset();
    yController.reset();
    angleController.reset(RobotState.getGlobalPose().getRotation().getRadians());
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
