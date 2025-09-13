package frc.robot.subsystems.superstructure;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;



//This ONLY CALCULATES transitions between states

public class SuperstructureStateMachine{

    private Map<SuperstructureState, List<SuperstructureTransition>> graph = new HashMap<>();
    private Set<SuperstructureState> states = new HashSet<>();
    private List<SuperstructureTransition> transitions = new ArrayList<>();

    private SuperstructureState currentState; //our current state 
    private SuperstructureState targetState; //the overall state we want to go to
    private SuperstructureState currentTargetState; //the state we have to go through to get to targetState

    private boolean isTransitioning = false;

    private List<SuperstructureTransition>[][] precomputedPaths;
    private Map<String, Double> transitionCostMap = new HashMap<>();


    private String getTransitionKey(SuperstructureState from, SuperstructureState to) {
        return from.name() + "->" + to.name();
    }

    public double getTransitionCost(SuperstructureState from, SuperstructureState to) {
        return transitionCostMap.getOrDefault(getTransitionKey(from, to), 1.0);
    }

    private void loadTransitionCosts() {
        transitionCostMap.clear();
        File costFile = new File(Filesystem.getDeployDirectory(), "transition_costs.txt");
        try (BufferedReader br = new BufferedReader(new FileReader(costFile))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 3) {
                    String key = parts[0] + "->" + parts[1];
                    double duration = Double.parseDouble(parts[2]);
                    transitionCostMap.put(key, duration);
                    Logger.recordOutput("Using Transition Costs", true);
                }
            }
        } catch (IOException e) {
            Logger.recordOutput(
                    "Superstructure/Error", "Failed to load transition costs: " + e.getMessage());
            Logger.recordOutput("Using Transition Costs", false);
        }
    }

    public void autoGenerateTransitions() {
        for (SuperstructureState from : SuperstructureState.values()) {
            for (SuperstructureState to : from.getAllowedStates()) {
                double cost = getTransitionCost(from, to);
                addTransition(
                        new SuperstructureTransition(from, to, cost, false));
            }
        }
    }

    @SuppressWarnings("unchecked")
    private void precomputeAllPaths() {
        int numStates = SuperstructureState.values().length;
        precomputedPaths = new ArrayList[numStates][numStates];

        for (SuperstructureState from : SuperstructureState.values()) {
            for (SuperstructureState to : SuperstructureState.values()) {
                if (from.equals(to)) {
                    precomputedPaths[from.ordinal()][to.ordinal()] = new ArrayList<>();
                } else {
                    precomputedPaths[from.ordinal()][to.ordinal()] =
                            computeTransitionPaths(from, to);
                }
            }
        }
    }

    public SuperstructureStateMachine(){
        this.currentState = null;
        this.targetState = null;
        this.currentTargetState = null;
        loadTransitionCosts();
        autoGenerateTransitions();
        precomputeAllPaths();
    }

    public void addTransition(SuperstructureTransition transition){
        states.add(transition.getFromState());
        states.add(transition.getToState());
        transitions.add(transition);
    }

    public List<SuperstructureTransition> computeTransitionPaths(SuperstructureState from, SuperstructureState to){
        
    }
    





    public SuperstructureState getCurrentState(){
        return currentState;
    }

    public SuperstructureState getTargetState(){
        return targetState;
    }

    public SuperstructureState getCurrentTargetState(){
        return currentTargetState;
    }

    public void setCurrentState(SuperstructureState state){
        if (!states.contains(state)){
            throw new IllegalArgumentException("State not registered: " + state);
        }
        currentState = state;
    }

    public void setTargetState(SuperstructureState state){
        if (!states.contains(state)){
            throw new IllegalArgumentException("State not registered: " + state);
        }
        targetState = state;
    }









}
