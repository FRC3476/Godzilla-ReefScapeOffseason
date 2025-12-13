package frc.robot.arbitraryTriggers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.Rumble;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.ClawOld;
import frc.robot.subsystems.intake.IntakeOld;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import java.util.Set;

public class ArbitraryTriggers {
  private final RobotContainer container;
  private final Superstructure superstructure;
  private final IntakeOld intake;
  private final ClawOld claw;
  private final CommandXboxController controller;
  private final RobotState robotState;
  private final Elevator elevator;

  public ArbitraryTriggers(
      RobotContainer container, CommandXboxController controller, RobotState robotState) {
    this.container = container;
    this.controller = controller;
    this.robotState = robotState;
    superstructure = container.getSuperStructure();
    intake = container.getIntake();
    claw = container.getClaw();
    elevator = container.getElevator();
    configureTriggers();
  }

  private void configureTriggers() {
    // feeder.dejamTrigger.onTrue(intake.dejamFeeder());
    // elevator.elevatorObjectTrigger.onTrue(elevator.dejamElevator()); // unused since
    // elevatorObjectTrigger is permanently false

    // recommended but untested
    claw.exhaustedCoral()
        .debounce(0.1)
        .onTrue(Commands.runOnce(() -> CoralStateTracker.forceSet(CoralPosition.NONE)));

    Trigger exhaustedAlgaeTrigger =
        new Trigger(() -> claw.getClawState() == ClawState.SCORING_ALGAE && !RobotState.hasAlgae());

    exhaustedAlgaeTrigger
        .debounce(0.5)
        .onTrue(
            Commands.either(
                claw.setClawStateCommand(ClawState.INTAKING_CORAL),
                claw.setClawStateCommand(ClawState.IDLE),
                () -> CoralStateTracker.hasCoral()));

    Trigger autoPreScoreTrigger =
        new Trigger(
            () ->
                CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR
                    && RobotState.getSuperstructureState() == SuperstructureState.STOW
                    && RobotState.getSuperstructureTargetState() == SuperstructureState.STOW);

    autoPreScoreTrigger
        .debounce(0.25)
        .onTrue(
            Commands.defer(
                () -> {
                  switch (robotState.getStoredScorePosition().getCoralScoreLevel()) {
                    case L1:
                      return superstructure
                          .setStateCommand(SuperstructureState.L1_AIM, "PRE_SCORE_L1")
                          .asProxy();
                    case L2:
                      return superstructure
                          .setStateCommand(SuperstructureState.L2_AIM, "PRE_SCORE_L2")
                          .asProxy();
                    case L3:
                      return superstructure
                          .setStateCommand(SuperstructureState.L3_AIM, "PRE_SCORE_L3")
                          .asProxy();
                    case L4:
                      return superstructure
                          .setStateCommand(SuperstructureState.L4_PRESCORE, "PRE_SCORE_L4")
                          .asProxy();
                    default:
                      return Commands.none();
                  }
                },
                Set.of()));

    //
    Trigger autoStowAlgaeTrigger =
        new Trigger(
            () ->
                RobotState.hasAlgae()
                    && RobotState.getSuperstructureState()
                        == SuperstructureState.INTAKE_ALGAE_GROUND);

    autoStowAlgaeTrigger
        .debounce(0.2)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Auto Stow Algae"));

    Trigger hasAlgaeHaptics = new Trigger(() -> RobotState.hasAlgae());
    Trigger hasCoralHaptics =
        new Trigger(
            () -> CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR);

    hasAlgaeHaptics.onTrue(new Rumble(controller, 0.5, 0.5, RumbleType.kBothRumble));

    hasAlgaeHaptics.onFalse(
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0)));

    hasCoralHaptics.onTrue(new Rumble(controller, 0.5, 0.5, RumbleType.kBothRumble));

    hasCoralHaptics.onFalse(
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0)));

    Command dejamCommand =
        intake
            .setIntakeStateCommand(IntakeState.REJECT_CORAL)
            .asProxy()
            .andThen(new WaitCommand(0.1)) // PREVIOUSLY 0.2
            .andThen(intake.setIntakeStateCommand(IntakeState.INTAKE).asProxy());

    Command intakeDejamCommand =
        intake
            .setIntakeStateCommand(IntakeState.IDLE)
            .asProxy()
            .andThen(new WaitCommand(0.1)) // PREVIOUSLY 0.2
            .andThen(intake.setIntakeStateCommand(IntakeState.INTAKE).asProxy());

    CoralStateTracker.isStuckAtFrontFeederTrigger().onTrue(dejamCommand);

    CoralStateTracker.isStuckAtIntakeTrigger().onTrue(intakeDejamCommand);

    intake.rejectCoralIntakeTrigger.onTrue(
        intake.setIntakeStateCommand(IntakeState.REJECT_INTAKE_CORAL));

    intake.rejectCoralIntakeTrigger.onFalse(intake.setIntakeStateCommand(IntakeState.IDLE));

    intake.rejectCoralIntakeAndFeederTrigger.onTrue(
        intake.setIntakeStateCommand(IntakeState.REJECT_CORAL));
    intake.rejectCoralIntakeAndFeederTrigger.onFalse(
        intake.setIntakeStateCommand(IntakeState.IDLE));
  }
}
