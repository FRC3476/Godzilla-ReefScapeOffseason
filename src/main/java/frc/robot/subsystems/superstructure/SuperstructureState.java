package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.RobotContainer;
import frc.robot.util.Util;
import java.util.EnumSet;
import java.util.Set;
import java.util.function.Function;

// This stores what subsystem values are in each state

public enum SuperstructureState {
  STOW(
      Constants.SuperstructureConstants.STOW_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_ENDEFFECTOR_ROTATION_ROTATIONS,
      container -> new ParallelCommandGroup(container.setIntakeStateCommand(IntakeState.STOW))),
  STOW_CORAL(
      Constants.SuperstructureConstants.STOW_CORAL_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS),
  STOW_ALGAE(
      Constants.SuperstructureConstants.STOW_ALGAE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_ALGAE_ENDEFFECTOR_ROTATION_ROTATIONS),
  INTAKE_CORAL(
      Constants.SuperstructureConstants.INTAKE_CORAL_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.INTAKE_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS,
      container -> new ParallelCommandGroup(container.setIntakeStateCommand(IntakeState.INTAKE))),
  INTAKE_CORAL_L1(
      Constants.SuperstructureConstants.INTAKE_CORAL_L1_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.INTAKE_CORAL_L1_ENDEFFECTOR_ROTATION_ROTATIONS,
      container ->
          new ParallelCommandGroup(container.setIntakeStateCommand(IntakeState.INTAKE_L1))),
  INTAKE_ALGAE_GROUND(
      Constants.SuperstructureConstants.ALGAE_GROUND_INTAKE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.ALGAE_GROUND_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS),
  FEED(
      Constants.SuperstructureConstants.FEED_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.FEED_ENDEFFECTOR_ROTATION_ROTATIONS),
  L1_PIVOT(
      Constants.SuperstructureConstants.L1_PIVOT_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L1_PIVOT_ENDEFFECTOR_ROTATION_ROTATIONS),
  L2_AIM(
      Constants.SuperstructureConstants.L2_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L2_AIM_ENDEFFECTOR_ROTATION_ROTATIONS),
  L3_AIM(
      Constants.SuperstructureConstants.L3_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L3_AIM_ENDEFFECTOR_ROTATION_ROTATIONS),
  L4_AIM(
      Constants.SuperstructureConstants.L4_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L4_AIM_ENDEFFECTOR_ROTATION_ROTATIONS),
  L1_SCORE(
      Constants.SuperstructureConstants.L1_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L1_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS),
  L2_SCORE(
      Constants.SuperstructureConstants.L2_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L2_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS),
  L3_SCORE(
      Constants.SuperstructureConstants.L3_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L3_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS),
  L4_SCORE(
      Constants.SuperstructureConstants.L4_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L4_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS),
  L2_FADEAWAY(
      Constants.SuperstructureConstants.L2_FADEAWAY_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L2_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS),
  L3_FADEAWAY(
      Constants.SuperstructureConstants.L3_FADEAWAY_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L3_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS),
  L4_FADEAWAY(
      Constants.SuperstructureConstants.L4_FADEAWAY_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L4_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS),
  ALGAE_HIGH_INTAKE(
      Constants.SuperstructureConstants.ALGAE_HIGH_INTAKE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.ALGAE_HIGH_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS),
  ALGAE_LOW_INTAKE(
      Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS),
  PROCESSOR_AIM(
      Constants.SuperstructureConstants.PROCESSOR_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.PROCESSOR_AIM_ENDEFFECTOR_ROTATION_ROTATIONS),
  BARGE_AIM_CENTER(
      Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.BARGE_AIM_CENTER_ENDEFFECTOR_ROTATION_ROTATIONS),
  BARGE_AIM_FORWARD(
      Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_ROTATIONS),
  BARGE_AIM_BACKWARD(
      Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.BARGE_AIM_BACKWARD_ENDEFFECTOR_ROTATION_ROTATIONS);

  private final double elevatorHeight;
  private final double endEffectorRotation;

  private final Function<RobotContainer, Command> commandSupplier;

  SuperstructureState(double elevatorHeight, double endEffectorRotation) {
    this.elevatorHeight = elevatorHeight;
    this.endEffectorRotation = endEffectorRotation;
    this.commandSupplier =
        (container) ->
            new ParallelCommandGroup(
                container.moveElevatorCommand(elevatorHeight),
                container.moveEndEffectorCommand(endEffectorRotation));
  }

  SuperstructureState(
      double elevatorHeight,
      double endEffectorRotation,
      Function<RobotContainer, Command> command) {
    this.elevatorHeight = elevatorHeight;
    this.endEffectorRotation = endEffectorRotation;
    this.commandSupplier = command;
  }

  public double getElevatorHeight() {
    return this.elevatorHeight;
  }

  public double getEndEffectorRotation() {
    return this.endEffectorRotation;
  }

  public Command getCommand(RobotContainer container) {
    if (commandSupplier == null) {
      return Commands.none();
    }
    return this.commandSupplier.apply(container);
  }

  public boolean isCoralState() {
    switch (this) {
      case STOW_CORAL,
          INTAKE_CORAL,
          INTAKE_CORAL_L1,
          L1_PIVOT,
          L2_AIM,
          L3_AIM,
          L4_AIM,
          L1_SCORE,
          L2_SCORE,
          L3_SCORE,
          L4_SCORE:
        return true;
      default:
        return false;
    }
  }

  // return a set of all the states you can go to from this state
  @SuppressWarnings("unchecked")
  public Set<SuperstructureState> getAllowedDestinationStates() {
    switch (this) {
      case STOW:
        return Util.mergeSets(outStates(), lowStates());
      case STOW_CORAL:
        return Util.mergeSets(outStates(), lowStates());
      case STOW_ALGAE:
        return Util.mergeSets(outStates(), lowStates());
      case INTAKE_CORAL:
        return Util.mergeSets(outStates(), lowStates());
      case INTAKE_CORAL_L1:
        return Util.mergeSets(outStates(), lowStates());
      case FEED:
        return Util.mergeSets(outStates(), lowStates());
      case L1_PIVOT:
        return Util.mergeSets(outStates(), lowStates());
      case L2_AIM:
        return Util.mergeSets(outStates(), lowStates(), topStates());
      case L3_AIM:
        return Util.mergeSets(outStates(), lowStates(), topStates());
      case L4_AIM:
        return Util.mergeSets(outStates(), topStates());
      case L1_SCORE:
        return Util.mergeSets(outStates(), lowStates());
      case L2_SCORE:
        return Util.mergeSets(outStates(), lowStates(), topStates());
      case L3_SCORE:
        return Util.mergeSets(outStates(), lowStates(), topStates());
      case L4_SCORE:
        return Util.mergeSets(outStates(), topStates());
      case ALGAE_HIGH_INTAKE:
        return Util.mergeSets(outStates(), lowStates(), topStates());
      case ALGAE_LOW_INTAKE:
        return Util.mergeSets(outStates(), lowStates(), topStates());
      case PROCESSOR_AIM:
        return Util.mergeSets(outStates(), lowStates(), topStates());
      case BARGE_AIM_CENTER:
        return Util.mergeSets(outStates(), topStates());
      case BARGE_AIM_FORWARD:
        return Util.mergeSets(outStates(), topStates());
      case BARGE_AIM_BACKWARD:
        return Util.mergeSets(outStates(), topStates());
      default:
        return EnumSet.of(STOW);
    }
  }

  public Set<SuperstructureState> lowStates() {
    return EnumSet.of(
        STOW, STOW_ALGAE, STOW_CORAL, INTAKE_CORAL, INTAKE_CORAL_L1, FEED, L1_PIVOT, L1_SCORE);
  }

  public Set<SuperstructureState> outStates() {
    return EnumSet.of(
        L2_AIM, L3_AIM, L2_SCORE, L3_SCORE, ALGAE_HIGH_INTAKE, ALGAE_LOW_INTAKE, PROCESSOR_AIM);
  }

  public Set<SuperstructureState> topStates() {
    return EnumSet.of(L4_AIM, L4_SCORE, BARGE_AIM_CENTER, BARGE_AIM_FORWARD, BARGE_AIM_BACKWARD);
  }
}
