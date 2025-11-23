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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.Led.DEFAULT_LED_STATE;
import frc.robot.subsystems.superstructure.SuperstructureState;
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
  private final Led led;
  private final Supplier<Pose2d> targetPoseSupplier;
  Debouncer debouncer = new Debouncer(0.1);

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
   * @param led The LED subsystem to control
   * @param targetPoseSupplier Supplier that provides the target pose to drive to
   */
  public DriveToPoseAutopilotCommand(
      RobotContainer container, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = container.getDrive();
    this.led = container.getLed();
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
   * @param led The LED subsystem to control
   * @param targetPose The target pose to drive to
   */
  public DriveToPoseAutopilotCommand(RobotContainer container, Pose2d targetPose) {
    this(container, () -> targetPose);
  }

  @Override
  public void initialize() {
    isAtTarget = false;
    RobotState.setReadyToScore(false);
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
    isAtTarget = debouncer.calculate(autopilot.atTarget(currentPose, target));

    // Apply field-relative velocities with FieldCentricFacingAngle
    // With ForwardPerspective set to BlueAlliance, this should match AutoPilot's coordinate system
    // CRITICAL: Always send control command, but with 0 velocities when at target
    // This prevents oscillation from the robot coasting after reaching target
    double vx = isAtTarget ? 0.0 : result.vx().in(MetersPerSecond);
    double vy = isAtTarget ? 0.0 : result.vy().in(MetersPerSecond);

    drive.setControl(
        facingAngleRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withTargetDirection(result.targetAngle()));

    // Turn on LEDs when at the right position and in a coral scoring state
    // If in L4_AIM, do that only if the robot is stable
    if (isAtTarget
        && RobotState.getSuperstructureState() == RobotState.getSuperstructureTargetState()
        && RobotState.getSuperstructureState().isCoralScoringState()
        && (RobotState.getSuperstructureState() != SuperstructureState.L4_AIM
            || drive.isRobotStable())) {
      led.setLedState(DEFAULT_LED_STATE.GARAGE_DRIVE_ALIGNED);
    }

    // Set ready to score when at target and in appropriate state
    if (RobotState.getSuperstructureState().isCoralScoringState()
        && RobotState.getSuperstructureState() == RobotState.getSuperstructureTargetState()
        && (RobotState.getSuperstructureState() != SuperstructureState.L4_AIM
            || (drive.isRobotStable() && isAtTarget))) {
      RobotState.setReadyToScore(true);
    }

    // Log telemetry
    Logger.recordOutput("Commands/" + getName() + "/CurrentPose", currentPose);
    Logger.recordOutput("Commands/" + getName() + "/TargetPose", targetPose);
    Logger.recordOutput("Commands/" + getName() + "/OffsetX", offsetX);
    Logger.recordOutput("Commands/" + getName() + "/OffsetY", offsetY);
    Logger.recordOutput("Commands/" + getName() + "/DistanceToTarget", distanceToTarget);
    Logger.recordOutput("Commands/" + getName() + "/RobotRelativeSpeeds", robotRelativeSpeeds);
    Logger.recordOutput("Commands/" + getName() + "/AutoPilotVelX", result.vx());
    Logger.recordOutput("Commands/" + getName() + "/AutoPilotVelY", result.vy());
    Logger.recordOutput("Commands/" + getName() + "/AppliedVelX", vx);
    Logger.recordOutput("Commands/" + getName() + "/AppliedVelY", vy);
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

    led.setLedState(DEFAULT_LED_STATE.NONE);
    RobotState.setReadyToScore(false);

    Logger.recordOutput("Commands/" + getName() + "/Active", false);
    Logger.recordOutput("Commands/" + getName() + "/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomous() && isAtTarget;
  }

  // Note: No isFinished() override - command runs until button is released or interrupted
  // This matches the behavior of GarageDriveToPoseCommand

  /**
   * Returns a trigger that is true when the robot is at the target pose.
   *
   * @return A trigger that activates when the robot reaches the target
   */
  public Trigger atTarget() {
    return new Trigger(() -> isAtTarget);
  }
}
