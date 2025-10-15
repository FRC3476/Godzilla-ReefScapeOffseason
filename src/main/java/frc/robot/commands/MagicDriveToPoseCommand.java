package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.CompTunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.PoseUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MagicDriveToPoseCommand extends Command {
  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD,
          new TrapezoidProfile.Constraints(
              (Constants.DriveConstants.kDriveMaxSpeed),
              Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD,
          new TrapezoidProfile.Constraints(
              (Constants.DriveConstants.kDriveMaxSpeed),
              Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared));
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveConstants.ANGLE_KP,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              DriveConstants.kDriveMaxAngularRate, DriveConstants.ANGLE_MAX_ACCELERATION));

  private double ffMinRadius = 0.0, ffMaxRadius = 0.1;

  private final Supplier<Pose2d> targetPoseSupplier;
  private final DriveSubsystem drive;

  private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  private final boolean unending;

  public MagicDriveToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    this(drive, targetPoseSupplier, false);
  }

  public MagicDriveToPoseCommand(
      DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier, boolean unending) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    xController.setTolerance(Units.inchesToMeters(0.5));
    yController.setTolerance(Units.inchesToMeters(0.5));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(0.5));
    this.unending = unending;
  }

  @Override
  public void initialize() {
    // arm center is the same as the robot center when stowed, so can use field to
    // robot

    xController.reset(0);
    yController.reset(0);
    angleController.reset(0);
  }

  @Override
  public void execute() {
    Pose2d robotPose = RobotState.getGlobalPose();
    Pose2d originalPose2d = targetPoseSupplier.get();
    Pose2d targetPose =
        originalPose2d.rotateAround(originalPose2d.getTranslation(), Rotation2d.kPi);

    double currentDistance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

    double currentDegrees =
        Math.abs(robotPose.getRotation().minus(targetPose.getRotation()).getDegrees());
    double rot_ffScaler = MathUtil.clamp(currentDegrees / 10, 0.0, 10.0);

    Logger.recordOutput("Commands/" + getName() + "/targetPose", targetPose);

    Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

    double perpendicularError = PoseUtils.getPerpendicularError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/PerpendicularError", perpendicularError);

    double parallelError = PoseUtils.getParallelError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/ParallelError", parallelError);

    double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();
    Logger.recordOutput("Commands/" + getName() + "/ThetaError", thetaError);

    double parallelSpeed =
        xController.getSetpoint().velocity * ffScaler
            + xController.calculate(-parallelError, 0)
            + Math.copySign(
                CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.02,
                xController.calculate(-parallelError, 0));
    parallelSpeed = !xController.atSetpoint() ? parallelSpeed : 0;
    Logger.recordOutput("Commands/" + getName() + "/parallelSpeed", parallelSpeed);

    double perpendicularSpeed =
        yController.getSetpoint().velocity * ffScaler
            + yController.calculate(-perpendicularError, 0)
            + Math.copySign(
                CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.02,
                xController.calculate(-parallelError, 0));

    if (Math.abs(thetaError) < 0.1) {
      perpendicularSpeed = !yController.atSetpoint() ? perpendicularSpeed : 0;
    } else {
      perpendicularSpeed = 0;
    }
    Logger.recordOutput("Commands/" + getName() + "/perpendicularSpeed", perpendicularSpeed);

    double angularSpeed =
        angleController.getSetpoint().velocity * rot_ffScaler
            + angleController.calculate(thetaError, 0)
            + Math.copySign(1.5, angleController.calculate(thetaError, 0));
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;
    Logger.recordOutput("Commands/" + getName() + "/angularSpeed", angularSpeed);

    double desiredTranslationSpeed = Math.hypot(perpendicularSpeed, parallelSpeed);
    double allowableTranslationSpeed =
        MathUtil.clamp(
            desiredTranslationSpeed, 0.0, CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    perpendicularSpeed = perpendicularSpeed * allowableTranslationSpeed / desiredTranslationSpeed;
    parallelSpeed = parallelSpeed * allowableTranslationSpeed / desiredTranslationSpeed;

    ChassisSpeeds speeds = new ChassisSpeeds(perpendicularSpeed, parallelSpeed, angularSpeed);

    drive.setControl(robotSpeeds.withSpeeds(speeds));
  }

  @Override
  public void end(boolean interrupt) {
    xController.reset(0);
    yController.reset(0);
    angleController.reset(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint()
        && yController.atSetpoint()
        && angleController.atSetpoint()
        && !unending;
  }
}
