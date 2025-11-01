package frc.robot.humanControls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Field.FieldConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GarageDriveToPoseCommand;
import frc.robot.commands.ParallelDriveCommand;
import frc.robot.commands.Rumble;
import frc.robot.subsystems.climb.ClimbRoller;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.PoseUtils;
import java.util.Set;

public class DriverControls {
  private final RobotContainer container;
  private final DriveSubsystem drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Claw claw;
  private final CommandXboxController controller;
  private final ClimbRoller climbRoller;
  private final RobotState robotState;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private Command scoreCommand;
  private Command scoreCommandNoRumble;

  public DriverControls(
      RobotContainer container, CommandXboxController controller, RobotState robotState) {
    this.container = container;
    this.controller = controller;
    this.robotState = robotState;
    drive = container.getDrive();
    superstructure = container.getSuperStructure();
    intake = container.getIntake();
    claw = container.getClaw();
    climbRoller = container.getClimbRoller();
    elevator = container.getElevator();
    endEffector = container.getEndEffector();
    configureXboxBindings();
  }

  /** Use this method to define your button->command mappings. */
  private void configureXboxBindings() {

    // Lock to angle when button is held
    controller
        .b()
        .whileTrue(
            DriveCommands.driveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> {
                  if (climbRoller.getClimbing()) {
                    return FieldUtils.isRedAlliance()
                        ? Rotation2d.fromDegrees(ClimbConstants.CLIMB_ANGLE_SNAP)
                        : Rotation2d.fromDegrees(ClimbConstants.CLIMB_ANGLE_SNAP)
                            .plus(Rotation2d.k180deg);
                  } else if (RobotState.hasAlgae()) {
                    return RobotState.getGlobalPose().getRotation().getCos() < 0
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero;
                  } else {
                    return FieldUtils.getClosestReef().getPose().getRotation();
                  }
                }));

    // // Auto Align
    controller
        .a()
        .whileTrue(
            Commands.either(
                Commands.defer(
                    () -> {
                      Rotation2d targetRotation;
                      int controllerDirMultiplier;
                      if (RobotState.getGlobalPose().getRotation().getCos() > 0) {
                        controllerDirMultiplier = -1;
                        targetRotation = Rotation2d.kZero;
                      } else {
                        controllerDirMultiplier = 1;
                        targetRotation = Rotation2d.k180deg;
                      }
                      if (FieldUtils.isOnRedSide()) {
                        return new ParallelDriveCommand(
                            drive,
                            () ->
                                new Pose2d(
                                    FieldConstants.halfFieldLength
                                        + (FieldUtils.facingBarge()
                                            ? DriveConstants
                                                .AUTO_ALIGN_BARGE_FORWARD_PERPENDICULAR_OFFSET
                                            : DriveConstants
                                                .AUTO_ALIGN_BARGE_BACKWARD_PERPENDICULAR_OFFSET),
                                    0,
                                    targetRotation),
                            () -> controllerDirMultiplier * controller.getLeftX());
                      } else {
                        return new ParallelDriveCommand(
                            drive,
                            () ->
                                new Pose2d(
                                    FieldConstants.halfFieldLength
                                        - (FieldUtils.facingBarge()
                                            ? DriveConstants
                                                .AUTO_ALIGN_BARGE_FORWARD_PERPENDICULAR_OFFSET
                                            : DriveConstants
                                                .AUTO_ALIGN_BARGE_BACKWARD_PERPENDICULAR_OFFSET),
                                    0,
                                    targetRotation),
                            () -> controllerDirMultiplier * controller.getLeftX());
                      }
                    },
                    Set.of(drive)),
                new GarageDriveToPoseCommand(
                    container,
                    () ->
                        PoseUtils.getPerpendicularOffsetPose(
                            FieldUtils.getChosenReefPole(() -> controller.getLeftX()).getPose(),
                            DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET)),
                () -> RobotState.hasAlgae()));

    // Ground algae intake
    controller
        .rightBumper()
        .onTrue(
            Commands.either(
                new Rumble(controller, 0.25, 0.5, RumbleType.kBothRumble),
                Commands.sequence(
                    superstructure.setStateCommand(
                        SuperstructureState.INTAKE_ALGAE_GROUND, "GROUND ALGAE"),
                    new WaitCommand(0.05),
                    claw.setClawStateCommand(ClawState.ALGAE)),
                claw::isCoralInClaw));

    controller
        .leftBumper()
        .whileTrue(intake.setIntakeStateCommand(IntakeState.REJECT_CORAL))
        .onFalse(intake.setIntakeStateCommand(IntakeState.IDLE));

    // Superstructure Stow
    controller
        .povLeft()
        .onTrue(
            Commands.parallel(
                Commands.either(
                    superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Stow Algae"),
                    superstructure.setStateCommand(SuperstructureState.STOW, "Stow"),
                    () -> RobotState.hasAlgae()),
                Commands.runOnce(
                    () -> {
                      scoreCommand.cancel();
                      scoreCommandNoRumble.cancel();
                    })));

    // Intake Stow
    controller
        .povRight()
        .onTrue(
            intake
                .setIntakeStateCommand(IntakeState.STOW)
                .asProxy()
                .alongWith(claw.setClawStateCommand(ClawState.IDLE).asProxy()));

    // Intake ground coral
    controller
        .leftTrigger(0.2)
        .onTrue(
            intake
                .setIntakeStateCommand(IntakeState.INTAKE)
                .asProxy()
                .alongWith(
                    Commands.either(
                        Commands.none(),
                        claw.setClawStateCommand(ClawState.INTAKING_CORAL).asProxy(),
                        () -> RobotState.hasAlgae())));

    // ALGAE DESCORE PREP
    controller
        .x()
        .onTrue(
            Commands.either(
                new Rumble(controller, 0.25, 0.5, RumbleType.kBothRumble),
                superstructure
                    .setStateCommand(
                        () -> robotState.getAlgaeDescoreSuperstructureState(), "Algae Descore Aim")
                    .asProxy()
                    .andThen(new WaitCommand(0.05))
                    .andThen(claw.setClawStateCommand(ClawState.ALGAE))
                    .asProxy(),
                claw::isCoralInClaw));

    // Score position Aim
    controller
        .y()
        .onTrue(
            Commands.either(
                new Rumble(controller, 0.25, 0.5, RumbleType.kBothRumble),
                superstructure
                    .setStateCommand(
                        () -> robotState.getSuperstructureScoreAimState(), "Aim Scoring")
                    .onlyIf(() -> claw.isCoralInClaw() || RobotState.hasAlgae())
                    .asProxy(),
                () ->
                    !robotState.isSafeToRaise()
                        && claw.isCoralInClaw()
                        && (robotState.getSuperstructureScoreAimState()
                                == SuperstructureState.L4_AIM
                            || robotState.getSuperstructureScoreAimState()
                                == SuperstructureState.L3_AIM)));

    scoreCommandNoRumble =
        Commands.sequence(
            new ConditionalCommand(
                new ConditionalCommand(
                    claw.setClawStateCommand(ClawState.SCORING_L1).asProxy(),
                    claw.setClawStateCommand(ClawState.SCORING).asProxy(),
                    () -> robotState.isL1Mode()),
                claw.setClawStateCommand(ClawState.SCORING_ALGAE).asProxy(),
                () ->
                    RobotState.getSuperstructureState() != null
                        && RobotState.getSuperstructureState().isCoralState()),
            new ConditionalCommand(
                new WaitUntilCommand(
                        () -> CoralStateTracker.getCurrentPosition() == CoralPosition.NONE)
                    .withTimeout(3),
                new WaitCommand(0.3),
                () -> RobotState.getSuperstructureState().isCoralState()),
            superstructure
                .setStateCommand(() -> robotState.getFadeawayState(), "Aim fade")
                .asProxy(),
            new WaitCommand(1).onlyIf(() -> robotState.getFadeawayState() == SuperstructureState.L1_FADEAWAY),
            Commands.either(
                claw.setClawStateCommand(ClawState.INTAKING_CORAL).asProxy(),
                claw.setClawStateCommand(ClawState.IDLE).asProxy(),
                () -> CoralStateTracker.hasCoral()),
            new ConditionalCommand(
                    new WaitUntilCommand(() -> robotState.isSafeToStow())
                        .andThen(
                            new ConditionalCommand(
                                    superstructure.setStateCommand(
                                        SuperstructureState.STOW, "STOW"),
                                    Commands.none(),
                                    () ->
                                        RobotState.getSuperstructureTargetState().isFadeawayState())
                                .asProxy()),
                    Commands.none(),
                    () -> RobotState.getSuperstructureState().isCoralState())
                .asProxy());

    scoreCommand =
        Commands.either(
            Commands.sequence(
                new ConditionalCommand(
                    new ConditionalCommand(
                        claw.setClawStateCommand(ClawState.SCORING_L1).asProxy(),
                        claw.setClawStateCommand(ClawState.SCORING).asProxy(),
                        () -> robotState.isL1Mode()),
                    claw.setClawStateCommand(ClawState.SCORING_ALGAE).asProxy(),
                    () ->
                        RobotState.getSuperstructureState() != null
                            && RobotState.getSuperstructureState().isCoralState()),
                new ConditionalCommand(
                    new WaitUntilCommand(
                            () -> CoralStateTracker.getCurrentPosition() == CoralPosition.NONE)
                        .withTimeout(3),
                    new WaitCommand(0.3),
                    () -> RobotState.getSuperstructureState().isCoralState()),
                superstructure
                    .setStateCommand(() -> robotState.getFadeawayState(), "Aim fade")
                    .asProxy(),
                new WaitCommand(1).onlyIf(() -> robotState.getFadeawayState() == SuperstructureState.L1_FADEAWAY),
                Commands.either(
                    claw.setClawStateCommand(ClawState.INTAKING_CORAL).asProxy(),
                    claw.setClawStateCommand(ClawState.IDLE).asProxy(),
                    () -> CoralStateTracker.hasCoral()),
                new ConditionalCommand(
                        new WaitUntilCommand(() -> robotState.isSafeToStow())
                            .andThen(
                                new ConditionalCommand(
                                        superstructure.setStateCommand(
                                            SuperstructureState.STOW, "STOW"),
                                        Commands.none(),
                                        () ->
                                            RobotState.getSuperstructureTargetState()
                                                .isFadeawayState())
                                    .asProxy()),
                        Commands.none(),
                        () -> RobotState.getSuperstructureState().isCoralState())
                    .asProxy()),
            new Rumble(controller, 0.25, 0.5, RumbleType.kBothRumble),
            () ->
                (drive.isRobotStable()
                        && RobotState.getSuperstructureState()
                            == RobotState.getSuperstructureTargetState()
                        && RobotState.getSuperstructureTargetState() == SuperstructureState.L4_AIM)
                    || (RobotState.getSuperstructureTargetState().isScoringState()
                        && RobotState.getSuperstructureTargetState() != SuperstructureState.L4_AIM)
                    || RobotState.getSuperstructureTargetState().isAlgaeScoringState());

    // Manual spit out game piece
    controller
        .rightTrigger(0.2) // check
        .and(new Trigger(() -> RobotState.isReadyToScore() || RobotState.hasAlgae()))
        .and(controller.a())
        .onTrue(scoreCommandNoRumble);

    controller
        .rightTrigger(0.2) // check
        // .and(new Trigger(() -> RobotState.isReadyToScore()))
        .and(controller.a().negate())
        .onTrue(scoreCommand);
  }
}
