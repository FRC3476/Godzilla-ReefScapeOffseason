package frc.robot.humanControls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralBranch;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GarageDriveToPoseCommand;
import frc.robot.subsystems.climb.ClimbRoller;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.PoseUtils;
import java.util.Map;

public class DriverControls {
  private final RobotContainer container;
  private final DriveSubsystem drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Claw claw;
  private final CommandXboxController controller;
  private final ClimbRoller climbRoller;
  private final RobotState robotState;

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
    configureXboxBindings();
  }

  /** Use this method to define your button->command mappings. */
  private void configureXboxBindings() {

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
                        ? Rotation2d.kCCW_90deg
                        : Rotation2d.kCW_90deg;
                  } else if (RobotState.hasAlgae()) {
                    return RobotState.getGlobalPose().getRotation().getCos() < 0
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero;
                  } else {
                    return FieldUtils.getClosestReef().getPose().getRotation();
                  }
                }));

    // // Auto Align
    Command autodriveCommand =
        new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(
                    CoralBranch.NONE,
                    new GarageDriveToPoseCommand(
                        drive,
                        () ->
                            PoseUtils.getPerpendicularOffsetPose(
                                FieldUtils.getClosestReefPole().getPose(),
                                DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))),
                Map.entry(
                    CoralBranch.LEFT,
                    new GarageDriveToPoseCommand(
                        drive,
                        () ->
                            PoseUtils.getPerpendicularOffsetPose(
                                FieldUtils.getClosestReef().leftPole.getPose(),
                                DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))),
                Map.entry(
                    CoralBranch.RIGHT,
                    new GarageDriveToPoseCommand(
                        drive,
                        () ->
                            PoseUtils.getPerpendicularOffsetPose(
                                FieldUtils.getClosestReef().rightPole.getPose(),
                                DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET)))),
            () -> robotState.getStoredScorePosition().getCoralBranch());

    controller
        .a()
        .onTrue(
            new SelectCommand<>(
                // Maps selector values to commands
                Map.ofEntries(
                    Map.entry(
                        CoralBranch.NONE,
                        new GarageDriveToPoseCommand(
                            drive,
                            () ->
                                PoseUtils.getPerpendicularOffsetPose(
                                    FieldUtils.getClosestReefPole().getPose(),
                                    DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))),
                    Map.entry(
                        CoralBranch.LEFT,
                        new GarageDriveToPoseCommand(
                            drive,
                            () ->
                                PoseUtils.getPerpendicularOffsetPose(
                                    FieldUtils.getClosestReef().leftPole.getPose(),
                                    DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))),
                    Map.entry(
                        CoralBranch.RIGHT,
                        new GarageDriveToPoseCommand(
                            drive,
                            () ->
                                PoseUtils.getPerpendicularOffsetPose(
                                    FieldUtils.getClosestReef().rightPole.getPose(),
                                    DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET)))),
                () -> robotState.getStoredScorePosition().getCoralBranch()));

    // Ground algae intake
    controller
        .rightBumper()
        .onTrue(
            Commands.parallel(
                superstructure.setStateCommand(
                    SuperstructureState.INTAKE_ALGAE_GROUND, "GROUND ALGAE"),
                claw.setClawStateCommand(ClawState.ALGAE)));

    // Processor Aim thingy
    controller
        .leftBumper()
        // .onTrue(superstructure.setStateCommand(SuperstructureState.PROCESSOR_AIM, "Aim
        // Processor"));
        .whileTrue(intake.setIntakeStateCommand(IntakeState.REJECT_CORAL))
        .onFalse(intake.setIntakeStateCommand(IntakeState.IDLE));

    // Superstructure Stow
    controller
        .povLeft()
        .onTrue(
            Commands.either(
                superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Stow Algae"),
                superstructure.setStateCommand(SuperstructureState.STOW, "Stow"),
                () -> RobotState.hasAlgae()));

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
            superstructure
                .setStateCommand(
                    () -> robotState.getAlgaeDescoreSuperstructureState(), "Algae Descore Aim")
                .asProxy()
                .alongWith(claw.setClawStateCommand(ClawState.ALGAE))
                .asProxy());

    // Score position Aim
    controller
        .y()
        .onTrue(
            superstructure
                .setStateCommand(() -> robotState.getSuperstructureScoreAimState(), "Aim Scoring")
                .onlyIf(() -> claw.isCoralInClaw() || RobotState.hasAlgae())
                .asProxy());

    // controller.back().onTrue(intake.setIntakeStateCommand(IntakeState.SCORING_PREP).asProxy());
    // controller.start().onTrue(intake.setIntakeStateCommand(IntakeState.SCORING).asProxy());
    // controller.povUp().onTrue(intake.setIntakeStateCommand(IntakeState.INTAKE_L1).asProxy());
    // Manual spit out game piece
    controller
        .rightTrigger(0.2) // check
        .onTrue(
            // Commands.either(
            //         intake.setIntakeStateCommand(IntakeState.SCORING).asProxy(),
            Commands.sequence(
                // new MagicDriveToPoseCommand(
                //     drive,
                //     () ->
                //         PoseUtils.getPerpendicularOffsetPose(
                //             FieldUtils.getClosestReefPole().getPose(), 0.7)),
                // new WaitCommand(0.2)
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
                    new WaitCommand(2.0),
                    () -> RobotState.getSuperstructureState().isCoralState()),
                superstructure
                    .setStateCommand(() -> robotState.getFadeawayState(), "Aim fade")
                    .asProxy(),
                Commands.either(
                    claw.setClawStateCommand(ClawState.INTAKING_CORAL).asProxy(),
                    claw.setClawStateCommand(ClawState.IDLE).asProxy(),
                    () -> CoralStateTracker.hasCoral()),
                new ConditionalCommand(
                        new WaitUntilCommand(() -> RobotState.isSafeToStow())
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
                    .asProxy())
            // () -> robotState.isL1Mode())
            // .asProxy()
            );
    controller.povUp().onTrue(Commands.none());
    controller.povDown().onTrue(Commands.none());
    controller.back().onTrue(Commands.none());
    controller.start().onTrue(Commands.none());
    controller.leftStick().onTrue(Commands.none());
    controller.rightStick().onTrue(Commands.none());
  }
}
