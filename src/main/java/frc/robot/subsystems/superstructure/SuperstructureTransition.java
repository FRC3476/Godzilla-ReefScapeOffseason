package frc.robot.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;



//Stores data related to any transition
public class SuperstructureTransition {
    private final SuperstructureState fromState;
    private final SuperstructureState toState;
    private final double transitionTime; // time in seconds for the transition
    private final boolean collision; // if true, this move is considered unsafe

    public SuperstructureTransition(
            SuperstructureState fromState,
            SuperstructureState toState,
            double transitionTime,
            boolean collision) {
        this.fromState = fromState;
        this.toState = toState;
        this.transitionTime = transitionTime;
        this.collision = collision;
    }

    public SuperstructureState getFromState() {
        return fromState;
    }

    public SuperstructureState getToState() {
        return toState;
    }

    public double getTransitionTime() {
        return transitionTime;
    }

    public boolean hasCollision() {
        return collision;
    }

    @Override
    public String toString() {
        return "Transition from "
                + fromState
                + " to "
                + toState
                + " (time: "
                + transitionTime
                + ", collision: "
                + collision
                + ")";
    }
}