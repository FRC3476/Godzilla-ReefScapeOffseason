package frc.robot.subsystems.superstructure;

import frc.robot.Constants;
import java.util.EnumSet;
import java.util.Set;

// This stores what subsystem values are in each state

public enum SuperstructureState {
  NONE(
      Constants.SuperstructureConstants.STOW_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_ENDEFFECTOR_ROTATION_RADIAN),
  STOW(
      Constants.SuperstructureConstants.STOW_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_ENDEFFECTOR_ROTATION_RADIAN),
  STOW_CORAL(
      Constants.SuperstructureConstants.STOW_CORAL_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_CORAL_ENDEFFECTOR_ROTATION_RADIAN),
  STOW_ALGAE(
      Constants.SuperstructureConstants.STOW_ALGAE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.STOW_ALGAE_ENDEFFECTOR_ROTATION_RADIAN),
  INTAKE_CORAL(
      Constants.SuperstructureConstants.INTAKE_CORAL_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.INTAKE_CORAL_ENDEFFECTOR_ROTATION_RADIAN),
  INTAKE_CORAL_L1(
      Constants.SuperstructureConstants.INTAKE_CORAL_L1_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.INTAKE_CORAL_L1_ENDEFFECTOR_ROTATION_RADIAN),
  FEED(
      Constants.SuperstructureConstants.FEED_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.FEED_ENDEFFECTOR_ROTATION_RADIAN),
  L1_PIVOT(
      Constants.SuperstructureConstants.L1_PIVOT_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L1_PIVOT_ENDEFFECTOR_ROTATION_RADIAN),
  L2_AIM(
      Constants.SuperstructureConstants.L2_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L2_AIM_ENDEFFECTOR_ROTATION_RADIAN),
  L3_AIM(
      Constants.SuperstructureConstants.L3_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L3_AIM_ENDEFFECTOR_ROTATION_RADIAN),
  L4_AIM(
      Constants.SuperstructureConstants.L4_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L4_AIM_ENDEFFECTOR_ROTATION_RADIAN),
  L1_SCORE(
      Constants.SuperstructureConstants.L1_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L1_SCORE_ENDEFFECTOR_ROTATION_RADIAN),
  L2_SCORE(
      Constants.SuperstructureConstants.L2_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L2_SCORE_ENDEFFECTOR_ROTATION_RADIAN),
  L3_SCORE(
      Constants.SuperstructureConstants.L3_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L3_SCORE_ENDEFFECTOR_ROTATION_RADIAN),
  L4_SCORE(
      Constants.SuperstructureConstants.L4_SCORE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L4_SCORE_ENDEFFECTOR_ROTATION_RADIAN),
  L2_FADEAWAY(
      Constants.SuperstructureConstants.L2_FADEAWAY_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L2_FADEAWAY_ENDEFFECTOR_ROTATION_RADIAN),
  L3_FADEAWAY(
      Constants.SuperstructureConstants.L3_FADEAWAY_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L3_FADEAWAY_ENDEFFECTOR_ROTATION_RADIAN),
  L4_FADEAWAY(
      Constants.SuperstructureConstants.L4_FADEAWAY_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.L4_FADEAWAY_ENDEFFECTOR_ROTATION_RADIAN),
  ALGAE_HIGH_INTAKE(
      Constants.SuperstructureConstants.ALGAE_HIGH_INTAKE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.ALGAE_HIGH_INTAKE_ENDEFFECTOR_ROTATION_RADIAN),
  ALGAE_LOW_INTAKE(
      Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_ENDEFFECTOR_ROTATION_RADIAN),
  PROCESSOR_AIM(
      Constants.SuperstructureConstants.PROCESSOR_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.PROCESSOR_AIM_ENDEFFECTOR_ROTATION_RADIAN),
  BARGE_AIM_CENTER(
      Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.BARGE_AIM_CENTER_ENDEFFECTOR_ROTATION_RADIAN),
  BARGE_AIM_FORWARD(
      Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_RADIAN),
  BARGE_AIM_BACKWARD(
      Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
      Constants.SuperstructureConstants.BARGE_AIM_BACKWARD_ENDEFFECTOR_ROTATION_RADIAN);

  private final double elevatorHeight;
  private final double endEffectorRotation;

  SuperstructureState(double elevatorHeight, double endEffectorRotation) {
    this.elevatorHeight = elevatorHeight;
    this.endEffectorRotation = endEffectorRotation;
  }

  public double getElevatorHeight() {
    return this.elevatorHeight;
  }

  public double getEndEffectorRotation() {
    return this.endEffectorRotation;
  }

  public boolean isCoralState() {
    switch (this) {
      case NONE:
        return false;
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

  public Set<SuperstructureState> getAllowedStates() {

    Set<SuperstructureState> allowedStates = EnumSet.allOf(SuperstructureState.class);
    allowedStates.remove(SuperstructureState.NONE);

    switch (this) {
      case NONE:
        return allowedStates;
      case STOW:
        return allowedStates;
      case STOW_CORAL:
        return allowedStates;
      case STOW_ALGAE:
        return allowedStates;
      case INTAKE_CORAL:
        return allowedStates;
      case INTAKE_CORAL_L1:
        return allowedStates;
      case FEED:
        return allowedStates;
      case L1_PIVOT:
        return allowedStates;
      case L2_AIM:
        return allowedStates;
      case L3_AIM:
        return allowedStates;
      case L4_AIM:
        return allowedStates;
      case L1_SCORE:
        return allowedStates;
      case L2_SCORE:
        return allowedStates;
      case L3_SCORE:
        return allowedStates;
      case L4_SCORE:
        return allowedStates;
      case ALGAE_HIGH_INTAKE:
        return allowedStates;
      case ALGAE_LOW_INTAKE:
        return allowedStates;
      case PROCESSOR_AIM:
        return allowedStates;
      case BARGE_AIM_CENTER:
        return allowedStates;
      case BARGE_AIM_FORWARD:
        return EnumSet.of(BARGE_AIM_CENTER);
      case BARGE_AIM_BACKWARD:
        return EnumSet.of(BARGE_AIM_CENTER);
      default:
        return allowedStates;
    }
  }
}
