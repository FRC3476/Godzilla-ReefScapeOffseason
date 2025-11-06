package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command that drives the robot to a target pose using the original AutoPilot library.
 *
 * <p>Uses TheRekrab's AutoPilot with proper velocity constraints. Applies velocities using
 * ApplyRobotSpeeds (like GarageDriveToPoseCommand) which is proven to work correctly.
 */
public class DriveToPoseAutopilotCommand extends Command {
  private APConstraints constraints;
  private APProfile profile;
  private final Autopilot autopilot;
  private final DriveSubsystem drive;
  private final Supplier<Pose2d> targetPoseSupplier;

  // Track if we're at target to ensure proper stopping
  private boolean isAtTarget = false;

  private final FieldCentricFacingAngle facingAngleRequest =
      new FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true)
          .withForwardPerspective(
              ForwardPerspectiveValue.BlueAlliance) // CRITICAL: Match AutoPilot coords!
          .withHeadingPID(
              DriveConstants.AUTOPILOT_HEADING_KP,
              DriveConstants.AUTOPILOT_HEADING_KI,
              DriveConstants.AUTOPILOT_HEADING_KD);

  /**
   * Creates a new DriveToPoseAutopilotCommand.
   *
   * @param drive The drive subsystem to control
   * @param targetPoseSupplier Supplier that provides the target pose to drive to
   */
  public DriveToPoseAutopilotCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;

    // Initialize AutoPilot with ALL constraints including velocity
    this.constraints =
        new APConstraints()
            .withVelocity(DriveConstants.kDriveMaxSpeed) // CRITICAL: Set max velocity!
            .withAcceleration(DriveConstants.AUTOPILOT_MAX_ACCELERATION)
            .withJerk(DriveConstants.AUTOPILOT_MAX_JERK);

    this.profile =
        new APProfile(constraints)
            .withErrorXY(Centimeters.of(DriveConstants.AUTOPILOT_ERROR_XY_METERS * 100))
            .withErrorTheta(Degrees.of(DriveConstants.AUTOPILOT_ERROR_THETA_DEGREES))
            .withBeelineRadius(
                Centimeters.of(DriveConstants.AUTOPILOT_BEELINE_RADIUS_METERS * 100));

    this.autopilot = new Autopilot(profile);

    addRequirements(drive);
  }

  /**
   * Creates a new DriveToPoseAutopilotCommand with a fixed target pose.
   *
   * @param drive The drive subsystem to control
   * @param targetPose The target pose to drive to
   */
  public DriveToPoseAutopilotCommand(DriveSubsystem drive, Pose2d targetPose) {
    this(drive, () -> targetPose);
  }

  @Override
  public void initialize() {
    isAtTarget = false;
    Logger.recordOutput("Commands/" + getName() + "/Active", true);
  }

  @Override
  public void execute() {
    // Get current robot state
    Pose2d currentPose = RobotState.getGlobalPose();
    ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();

    // Get target pose
    Pose2d targetPose = targetPoseSupplier.get();

    // Create AutoPilot target
    APTarget target = new APTarget(targetPose);

    // Calculate desired velocities using ORIGINAL AutoPilot
    APResult result = autopilot.calculate(currentPose, robotRelativeSpeeds, target);

    // Calculate offset for debugging
    double offsetX = targetPose.getX() - currentPose.getX();
    double offsetY = targetPose.getY() - currentPose.getY();
    double distanceToTarget = Math.hypot(offsetX, offsetY);

    // Check if we're at target
    isAtTarget = autopilot.atTarget(currentPose, target);

    // Apply field-relative velocities with FieldCentricFacingAngle
    // With ForwardPerspective set to BlueAlliance, this should match AutoPilot's coordinate system
    drive.setControl(
        facingAngleRequest
            .withVelocityX(result.vx().in(MetersPerSecond))
            .withVelocityY(result.vy().in(MetersPerSecond))
            .withTargetDirection(result.targetAngle()));

    // Log telemetry
    Logger.recordOutput("Commands/" + getName() + "/CurrentPose", currentPose);
    Logger.recordOutput("Commands/" + getName() + "/TargetPose", targetPose);
    Logger.recordOutput("Commands/" + getName() + "/OffsetX", offsetX);
    Logger.recordOutput("Commands/" + getName() + "/OffsetY", offsetY);
    Logger.recordOutput("Commands/" + getName() + "/DistanceToTarget", distanceToTarget);
    Logger.recordOutput("Commands/" + getName() + "/RobotRelativeSpeeds", robotRelativeSpeeds);
    Logger.recordOutput("Commands/" + getName() + "/FieldRelVelX", result.vx());
    Logger.recordOutput("Commands/" + getName() + "/FieldRelVelY", result.vy());
    Logger.recordOutput("Commands/" + getName() + "/CurrentRotation", currentPose.getRotation());
    Logger.recordOutput("Commands/" + getName() + "/TargetAngle", result.targetAngle());
    Logger.recordOutput("Commands/" + getName() + "/AtTarget", isAtTarget);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drive.setControl(
        facingAngleRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withTargetDirection(RobotState.getGlobalPose().getRotation()));

    Logger.recordOutput("Commands/" + getName() + "/Active", false);
    Logger.recordOutput("Commands/" + getName() + "/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    // Use the isAtTarget flag updated in execute()
    return isAtTarget;
  }

  /**
   * Returns a trigger that is true when the robot is at the target pose.
   *
   * @return A trigger that activates when the robot reaches the target
   */
  public Trigger atTarget() {
    return new Trigger(() -> isAtTarget);
  }
}
