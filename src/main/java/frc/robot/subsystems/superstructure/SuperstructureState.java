package frc.robot.subsystems.superstructure;

import java.util.EnumSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.RobotContainer;
import frc.robot.util.Util;

// This stores what subsystem values are in each state

public enum SuperstructureState {
  NONE(),
  CLIMB(
      Constants.SuperstructureConstants.CLIMB_ENDEFFECTOR_SAFE_ROTATIONS,
      Constants.SuperstructureConstants.STOW_ENDEFFECTOR_ROTATION_ROTATIONS),
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
  L2_AWAY_FROM_REEF(
      Constants.SuperstructureConstants.L2_AIM_AWAY_FROM_REEF_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L2_AIM_AWAY_FROM_REEF_ENDEFFECTOR_ROTATION_ROTATIONS),
  L3_AWAY_FROM_REEF(
      Constants.SuperstructureConstants.L3_AIM_AWAY_FROM_REEF_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L3_AIM_AWAY_FROM_REEF_ENDEFFECTOR_ROTATION_ROTATIONS),
  L4_AWAY_FROM_REEF(
      Constants.SuperstructureConstants.L4_AIM_AWAY_FROM_REEF_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L4_AIM_AWAY_FROM_REEF_ENDEFFECTOR_ROTATION_ROTATIONS),
  L1_FADEAWAY(
      Constants.SuperstructureConstants.L1_FADEAWAY_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L1_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS),
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

  public enum TransitionShortcutType {
    NONE,
    LOW_IN_TO_OUT,
    OUT_TO_HIGH_IN,
    OUT_TO_LOW_IN,
    HIGH_IN_TO_OUT
  }

  // run the guaranteed safe transition while it's unsafe to skip
  public Command getAsTransitionCommand(
      RobotContainer container, TransitionShortcutType shortcutType) {
    return Commands.select(
        Map.of(
            TransitionShortcutType.NONE,
            this.getCommand(container),
            // only be in transition if the elevator current position is less than the safe low
            // amount
            TransitionShortcutType.LOW_IN_TO_OUT,
            this.getCommand(container).onlyWhile(() -> !container.getEndEffector().isPivotSafe()),
            // only be in transition if the elevator current position is less than the safe low
            // amount
            TransitionShortcutType.OUT_TO_HIGH_IN,
            this.getCommand(container)
                .onlyWhile(
                    () ->
                        container.getElevator().getCurrentPosition()
                            < Constants.SuperstructureConstants
                                .HIGH_IN_SAFE_ELEVATOR_HEIGHT_INCHES),
            // only be in transition if the elevator current position is less than the safe low
            // amount
            TransitionShortcutType.OUT_TO_LOW_IN,
            this.getCommand(container)
                .onlyWhile(
                    () ->
                        container.getElevator().getCurrentPosition()
                            > Constants.SuperstructureConstants.LOW_IN_SAFE_ELEVATOR_HEIGHT_INCHES),
            // only be in transition if the elevator current position is less than the safe low
            // amount
            TransitionShortcutType.HIGH_IN_TO_OUT,
            this.getCommand(container).onlyWhile(() -> !container.getEndEffector().isPivotSafe())),
        () -> shortcutType);
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
          L1_FADEAWAY,
          L2_FADEAWAY,
          L3_FADEAWAY,
          L4_FADEAWAY,
          L2_AWAY_FROM_REEF,
          L3_AWAY_FROM_REEF,
          L4_AWAY_FROM_REEF:
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

    if (this == NONE) {
      return EnumSet.of(STOW);
    } else if (lowInStates().contains(this)) { // low in states
      return Util.mergeSets(lowOutStates(), lowInStates());
    } else if (lowOutStates().contains(this)) { // low out states
      return Util.mergeSets(lowOutStates(), lowInStates(), highOutStates(), middleOutStates());
    } else if (middleOutStates().contains(this)){ // Middle out states
      return Util.mergeSets(lowOutStates(), highOutStates(), middleOutStates());
    } else if (highOutStates().contains(this)) { // high out states
      return Util.mergeSets(lowOutStates(), highInStates(), highOutStates(), middleOutStates());
    } else if (highInStates().contains(this)) { // high in states
      return Util.mergeSets(highOutStates(), highInStates());
    }
    return EnumSet.of(NONE);
  }

  private Set<SuperstructureState> lowInStates() {
    return EnumSet.allOf(SuperstructureState.class).stream()
		.filter(SuperstructureState::isLowInFilter)
		.collect(Collectors.toCollection(() -> EnumSet.noneOf(SuperstructureState.class)));
  }

  private Set<SuperstructureState> lowOutStates() {
    return EnumSet.allOf(SuperstructureState.class).stream()
		.filter(SuperstructureState::isLowOutFilter)
		.collect(Collectors.toCollection(() -> EnumSet.noneOf(SuperstructureState.class)));  }

  private Set<SuperstructureState> middleOutStates() {
    return EnumSet.allOf(SuperstructureState.class).stream()
		.filter(SuperstructureState::isMiddleOutFilter)
		.collect(Collectors.toCollection(() -> EnumSet.noneOf(SuperstructureState.class)));
  }

  private Set<SuperstructureState> highOutStates() {
    return EnumSet.allOf(SuperstructureState.class).stream()
		.filter(SuperstructureState::isHighOutFilter)
		.collect(Collectors.toCollection(() -> EnumSet.noneOf(SuperstructureState.class)));
  }

  private Set<SuperstructureState> highInStates() {
    return EnumSet.allOf(SuperstructureState.class).stream()
		.filter(SuperstructureState::isHighInFilter)
		.collect(Collectors.toCollection(() -> EnumSet.noneOf(SuperstructureState.class)));  
  }

  public boolean isLowIn() {
    return lowInStates().contains(this);
  }

  public boolean isLowOut() {
    return lowOutStates().contains(this);
  }

  public boolean isMiddleOut() {
    return middleOutStates().contains(this);
  }

  public boolean isHighOut() {
    return highOutStates().contains(this);
  }

  public boolean isHighIn() {
    return highInStates().contains(this);
  }

  //superstructure filters
  private boolean isMiddleOutFilter() {
	  return this.getElevatorHeight() >= SuperstructureConstants.LOW_IN_SAFE_ELEVATOR_HEIGHT_INCHES
	  	&& this.getElevatorHeight() <= SuperstructureConstants.HIGH_IN_SAFE_ELEVATOR_HEIGHT_INCHES
	  	&& this.getEndEffectorRotation() >= EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS 
	  	&& this.getEndEffectorRotation() <= EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS;
  }

  private boolean isLowOutFilter() {
	  return this.getElevatorHeight() <= SuperstructureConstants.LOW_IN_SAFE_ELEVATOR_HEIGHT_INCHES
	  	&& this.getEndEffectorRotation() >= EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS 
	  	&& this.getEndEffectorRotation() <= EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS;
  }

  private boolean isHighOutFilter() {
	  return this.getElevatorHeight() >= SuperstructureConstants.HIGH_IN_SAFE_ELEVATOR_HEIGHT_INCHES
	  	&& this.getEndEffectorRotation() >= EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS 
	  	&& this.getEndEffectorRotation() <= EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS;
  }

  private boolean isHighInFilter(){
    return this.getElevatorHeight() >= SuperstructureConstants.HIGH_IN_SAFE_ELEVATOR_HEIGHT_INCHES
	  	&& this.getEndEffectorRotation() >= EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS;
  }

  private boolean isLowInFilter(){
    return this.getElevatorHeight() <= SuperstructureConstants.LOW_IN_SAFE_ELEVATOR_HEIGHT_INCHES
	  	&& this.getEndEffectorRotation() <= EndEffectorConstants.MIN_SAFE_ANGLE_ROTATIONS;
  }


}
