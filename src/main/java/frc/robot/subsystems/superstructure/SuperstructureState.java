package frc.robot.subsystems.superstructure;

import java.util.EnumSet;
import java.util.Set;

import frc.robot.Constants;

//This stores what subsystem values are in each state

public enum SuperstructureState {
    STOW(
        Constants.SuperstructureConstants.STOW_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.STOW_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.STOW_INTAKE_ROTATION_RADIAN
    ),
    STOW_CORAL(
        Constants.SuperstructureConstants.STOW_CORAL_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.STOW_CORAL_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.STOW_CORAL_INTAKE_ROTATION_RADIAN
    ),
    STOW_ALGAE(
        Constants.SuperstructureConstants.STOW_ALGAE_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.STOW_ALGAE_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.STOW_ALGAE_INTAKE_ROTATION_RADIAN
    ),
    INTAKE_CORAL(
        Constants.SuperstructureConstants.INTAKE_CORAL_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.INTAKE_CORAL_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.INTAKE_CORAL_INTAKE_ROTATION_RADIAN
    ),
    INTAKE_CORAL_L1(
        Constants.SuperstructureConstants.INTAKE_CORAL_L1_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.INTAKE_CORAL_L1_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.INTAKE_CORAL_L1_INTAKE_ROTATION_RADIAN,
        true
    ),
    FEED(
        Constants.SuperstructureConstants.FEED_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.FEED_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.FEED_INTAKE_ROTATION_RADIAN
    ),
    L1_PIVOT(
        Constants.SuperstructureConstants.L1_PIVOT_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L1_PIVOT_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L1_PIVOT_INTAKE_ROTATION_RADIAN
    ),
    L2_AIM(
        Constants.SuperstructureConstants.L2_AIM_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L2_AIM_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L2_AIM_INTAKE_ROTATION_RADIAN
    ),
    L3_AIM(
        Constants.SuperstructureConstants.L3_AIM_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L3_AIM_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L3_AIM_INTAKE_ROTATION_RADIAN
    ),
    L4_AIM(
        Constants.SuperstructureConstants.L4_AIM_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L4_AIM_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L4_AIM_INTAKE_ROTATION_RADIAN
    ),
    L1_SCORE(
        Constants.SuperstructureConstants.L1_SCORE_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L1_SCORE_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L1_SCORE_INTAKE_ROTATION_RADIAN
    ),
    L2_SCORE(
        Constants.SuperstructureConstants.L2_SCORE_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L2_SCORE_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L2_SCORE_INTAKE_ROTATION_RADIAN
    ),
    L3_SCORE(
        Constants.SuperstructureConstants.L3_SCORE_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L3_SCORE_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L3_SCORE_INTAKE_ROTATION_RADIAN
    ),
    L4_SCORE(
        Constants.SuperstructureConstants.L4_SCORE_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.L4_SCORE_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.L4_SCORE_INTAKE_ROTATION_RADIAN
    ),
    ALGAE_HIGH_INTAKE(
        Constants.SuperstructureConstants.ALGAE_HIGH_INTAKE_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.ALGAE_HIGH_INTAKE_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_INTAKE_ROTATION_RADIAN
    ),
    ALGAE_LOW_INTAKE(
        Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.ALGAE_LOW_INTAKE_INTAKE_ROTATION_RADIAN
    ),
    PROCESSOR_AIM(
        Constants.SuperstructureConstants.PROCESSOR_AIM_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.PROCESSOR_AIM_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.PROCESSOR_AIM_INTAKE_ROTATION_RADIAN
    ),
    BARGE_AIM_CENTER(
        Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.BARGE_AIM_CENTER_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.BARGE_AIM_INTAKE_ROTATION_RADIAN
    ),
    BARGE_AIM_FORWARD(
        Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.BARGE_AIM_INTAKE_ROTATION_RADIAN
    ),
    BARGE_AIM_BACKWARD(
        Constants.SuperstructureConstants.BARGE_AIM_ELEVATOR_HEIGHT_INCH,
        Constants.SuperstructureConstants.BARGE_AIM_BACKWARD_ENDEFFECTOR_ROTATION_RADIANS,
        Constants.SuperstructureConstants.BARGE_AIM_INTAKE_ROTATION_RADIAN
    );
    

    private final double elevatorHeight;
    private final double endEffectorRotation;
    private final double intakeRotation;
    private final boolean isIntakeBarDown;

    SuperstructureState(double elevatorHeight, double endEffectorRotation, double intakeRotation){
        this.elevatorHeight = elevatorHeight;
        this.endEffectorRotation = endEffectorRotation;
        this.intakeRotation = intakeRotation;
        this.isIntakeBarDown = false;
    }

    SuperstructureState(double elevatorHeight, double endEffectorRotation, double intakeRotation, boolean isIntakeBarDown){
        this.elevatorHeight = elevatorHeight;
        this.endEffectorRotation = endEffectorRotation;
        this.intakeRotation = intakeRotation;
        this.isIntakeBarDown = isIntakeBarDown;
    }

    public double getElevatorHeight(){
        return this.elevatorHeight;
    }

    public double getEndEffectorRotation(){
        return this.endEffectorRotation;
    }

    public double getIntakeRotation(){
        return this.intakeRotation;
    }

    public boolean getIntakeDown(){
        return this.isIntakeBarDown;
    }

    public boolean isCoralState(){
        switch (this){
            case STOW_CORAL, INTAKE_CORAL, INTAKE_CORAL_L1, 
            L1_PIVOT, L2_AIM, L3_AIM, L4_AIM,
            L1_SCORE, L2_SCORE, L3_SCORE, L4_SCORE:
                return true;
            default:
                return false;
        }
        
    }

    public Set<SuperstructureState> getAllowedStates(){
        
        Set<SuperstructureState> allowedStates = EnumSet.allOf(SuperstructureState.class);

        switch (this){
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
