package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PoseUtils;
import org.littletonrobotics.junction.Logger;

public class DriveToCoralCommand extends Command {
  private final ProfiledPIDController distanceController =
      new ProfiledPIDController(
          DriveConstants.DRIVE_TO_CORAL_KP,
          DriveConstants.DRIVE_TO_CORAL_KI,
          DriveConstants.DRIVE_TO_CORAL_KD,
          new TrapezoidProfile.Constraints(
              (Constants.DriveConstants.kDriveMaxSpeed / 2),
              Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared / 2));
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveConstants.ANGLE_KP,
          0.0,
          DriveConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.kDriveMaxAngularRate / 2, DriveConstants.ANGLE_MAX_ACCELERATION / 2));

  private final DriveSubsystem drive;
  private final Vision vision;

  private double ffMinRadius = 0.0, ffMaxRadius = 0.1;

  private final ApplyRobotSpeeds robotSpeeds =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  public DriveToCoralCommand(DriveSubsystem drive, Vision vision) {
    addRequirements(drive);

    this.drive = drive;
    this.vision = vision;
    distanceController.setTolerance(Units.inchesToMeters(0.5));
    angleController.setTolerance(Units.degreesToRadians(0.5));
  }

  public DriveToCoralCommand withJoystickRumble(Command rumbleCommand) {
    atSetpoint().onTrue(Commands.deferredProxy(() -> rumbleCommand));

    return this;
  }

  // Heavily inspired by 6328's `AutoAlignController` from 2024
  // https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/controllers/AutoAlignController.java#L135
  @Override
  public void execute() {
    if (vision.getCoralPose().isPresent()) {
      Pose2d robotPoseOriginal = RobotState.getGlobalPose();
      Logger.recordOutput("Align Offset Inches", Constants.kAlignOffset);
      Logger.recordOutput("Align OffsetFB Inches", Constants.kAlignOffsetFB);
      Pose2d robot =
          robotPoseOriginal.plus(
              new Transform2d(
                  Units.inchesToMeters(Constants.kAlignOffsetFB),
                  Units.inchesToMeters(Constants.kAlignOffset),
                  Rotation2d.kZero));

      Translation2d targetTranslation = vision.getCoralPose().get().getTranslation();
      Translation2d translationalError = robot.getTranslation().minus(targetTranslation);
      Rotation2d targetRotation =
          new Rotation2d(translationalError.getX(), translationalError.getY())
              .plus(Rotation2d.k180deg);
      Pose2d target =
          PoseUtils.getPerpendicularOffsetPose(
              new Pose2d(targetTranslation, targetRotation),
              DriveConstants.DRIVE_TO_CORAL_PERP_OFFSET_METERS);

      translationalError = robot.getTranslation().minus(target.getTranslation());

      double currentDistance = robot.getTranslation().getDistance(target.getTranslation());
      double ffScaler =
          MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
      Rotation2d angularError = robot.getRotation().minus(target.getRotation());

      // Magnitude of translational velocity, meaning that x & y are controlled together
      double translationalSpeed =
          distanceController.getSetpoint().velocity * ffScaler
              + MathUtil.clamp(
                  distanceController.calculate(translationalError.getNorm(), 0),
                  -DriveConstants.kDriveMaxSpeed,
                  DriveConstants.kDriveMaxSpeed);
      translationalSpeed = !distanceController.atGoal() ? translationalSpeed : 0;

      double anglularVelocity =
          angleController.getSetpoint().velocity * ffScaler
              + angleController.calculate(angularError.getRadians(), 0);
      anglularVelocity = !angleController.atGoal() ? anglularVelocity : 0;

      Translation2d velocity = new Translation2d(translationalSpeed, translationalError.getAngle());

      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              velocity.getX(), velocity.getY(), anglularVelocity, robot.getRotation());

      drive.setControl(robotSpeeds.withSpeeds(speeds));

      Logger.recordOutput("Commands/" + getName() + "/Robot", robot);
      Logger.recordOutput("Commands/" + getName() + "/Target", target);
      Logger.recordOutput("Commands/" + getName() + "/Error/Translational", translationalError);
      Logger.recordOutput("Commands/" + getName() + "/Error/Angular", angularError);
      Logger.recordOutput(
          "Commands/" + getName() + "/AtGoal/Translation", distanceController.atGoal());
      Logger.recordOutput("Commands/" + getName() + "/AtGoal/Angle", angleController.atGoal());
      Logger.recordOutput("Commands/" + getName() + "/Speeds/Translational", translationalSpeed);
      Logger.recordOutput("Commands/" + getName() + "/Speeds/ChassisSpeeds", speeds);
    }
  }

  @Override
  public void end(boolean interrupt) {
    drive.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds()));
    distanceController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint() {
    return new Trigger(() -> distanceController.atSetpoint() && angleController.atSetpoint());
  }

  // public Trigger canShoot(
  //     double distanceTolerance, double rotationTolerance, double velocityTolerance) {
  //   return new Trigger(
  //       () ->
  //           Math.abs(distanceController.getPositionError()) < distanceTolerance
  //               && Math.abs(angleController.getPositionError()) < rotationTolerance
  //               && Math.hypot(
  //                       drive.getChassisSpeeds().vxMetersPerSecond,
  //                       drive.getChassisSpeeds().vyMetersPerSecond)
  //                   < velocityTolerance);
  // }
  @Override
  public boolean isFinished() {
    return CoralStateTracker.getCurrentPosition() != CoralPosition.NONE;
  }
}
