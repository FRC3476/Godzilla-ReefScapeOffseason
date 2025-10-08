package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.Util;
import java.util.EnumSet;
import java.util.Set;
import java.util.function.Function;

// This stores what subsystem values are in each state

public enum SuperstructureState {
  NONE(),
  STOW(
      Constants.SuperstructureConstants.STOW_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_ENDEFFECTOR_ROTATION_ROTATIONS),
  STOW_CORAL(
      Constants.SuperstructureConstants.STOW_CORAL_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS),
  STOW_ALGAE(
      Constants.SuperstructureConstants.STOW_ALGAE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_ALGAE_ENDEFFECTOR_ROTATION_ROTATIONS),
  INTAKE_CORAL(
      Constants.SuperstructureConstants.INTAKE_CORAL_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.INTAKE_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS),
  INTAKE_CORAL_L1(
      Constants.SuperstructureConstants.INTAKE_CORAL_L1_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.INTAKE_CORAL_L1_ENDEFFECTOR_ROTATION_ROTATIONS),
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
  BARGE_AIM_BACKWARD(
      Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_ROTATIONS),
  BARGE_AIM_FORWARD(
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
                container.getElevator().moveElevatorCommand(() -> elevatorHeight),
                container.getEndEffector().moveEndEffectorCommand(() -> endEffectorRotation));
  }

  SuperstructureState(
      double elevatorHeight,
      double endEffectorRotation,
      Function<RobotContainer, Command> commandFunction) {
    this.elevatorHeight = elevatorHeight;
    this.endEffectorRotation = endEffectorRotation;
    this.commandSupplier =
        (container) ->
            new ParallelCommandGroup(
                container.getElevator().moveElevatorCommand(() -> elevatorHeight),
                container.getEndEffector().moveEndEffectorCommand(() -> endEffectorRotation),
                commandFunction.apply(container));
  }

  SuperstructureState() {
    this.elevatorHeight = 0;
    this.endEffectorRotation = 0;
    this.commandSupplier = (container) -> Commands.none();
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
      case STOW_CORAL, INTAKE_CORAL, INTAKE_CORAL_L1, L1_PIVOT, L2_AIM, L3_AIM, L4_AIM:
        return true;
      default:
        return false;
    }
  }

  public boolean isHandoffState() {
    switch (this) {
      case STOW, STOW_CORAL, INTAKE_CORAL, INTAKE_CORAL_L1, FEED:
        return true;
      default:
        return false;
    }
  }

  // return a set of all the states you can go to from this state
  @SuppressWarnings("unchecked")
  public Set<SuperstructureState> getAllowedDestinationStates() {
    switch (this) {
      case NONE:
        return EnumSet.of(STOW);
      case STOW,
          STOW_CORAL,
          STOW_ALGAE,
          INTAKE_CORAL,
          INTAKE_CORAL_L1,
          FEED,
          INTAKE_ALGAE_GROUND,
          L1_PIVOT: // low in states
        return Util.mergeSets(lowOutStates(), lowInStates());
      case L2_AIM,
          L3_AIM,
          L2_FADEAWAY,
          L3_FADEAWAY,
          ALGAE_LOW_INTAKE,
          PROCESSOR_AIM: // low out states
        return Util.mergeSets(lowOutStates(), lowInStates(), highOutStates());
      case L4_AIM, L4_FADEAWAY, ALGAE_HIGH_INTAKE, BARGE_AIM_BACKWARD: // high out states
        return Util.mergeSets(lowOutStates(), highInStates(), highOutStates());
      case BARGE_AIM_CENTER, BARGE_AIM_FORWARD:
        return Util.mergeSets(highOutStates(), highInStates()); // high in states
      default:
        return EnumSet.noneOf(SuperstructureState.class);
    }
  }

  public Set<SuperstructureState> lowInStates() {
    return EnumSet.of(
        STOW,
        STOW_ALGAE,
        STOW_CORAL,
        INTAKE_CORAL,
        INTAKE_CORAL_L1,
        FEED,
        L1_PIVOT,
        INTAKE_ALGAE_GROUND);
  }

  public Set<SuperstructureState> lowOutStates() {
    return EnumSet.of(PROCESSOR_AIM, L2_FADEAWAY, L2_AIM, ALGAE_LOW_INTAKE);
  }

  public Set<SuperstructureState> highOutStates() {
    return EnumSet.of(
        L3_AIM, ALGAE_HIGH_INTAKE, L3_FADEAWAY, L4_FADEAWAY, L4_AIM, BARGE_AIM_BACKWARD);
  }

  public Set<SuperstructureState> highInStates() {
    return EnumSet.of(BARGE_AIM_CENTER, BARGE_AIM_FORWARD);
  }
}
