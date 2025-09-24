package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.end_effector.EndEffector;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

// This ONLY CALCULATES transitions between states
public class SuperstructureStateMachine {

  private Map<SuperstructureState, List<SuperstructureTransition>> graph = new HashMap<>();
  private Set<SuperstructureState> states = new HashSet<>();
  private List<SuperstructureTransition> transitions = new ArrayList<>();

  private SuperstructureState currentState; // our current state
  private SuperstructureState targetState; // the overall state we want to go to
  private SuperstructureState
      currentTargetState; // the state we have to go through to get to targetState

  private boolean isTransitioning = false;

  private List<SuperstructureTransition>[][] precomputedPaths;
  private Map<String, Double> transitionCostMap = new HashMap<>();

  private EndEffector endEffector;

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
        addTransition(new SuperstructureTransition(from, to, cost, false));
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
          precomputedPaths[from.ordinal()][to.ordinal()] = computeTransitionPaths(from, to);
        }
      }
    }
  }

  public SuperstructureStateMachine(EndEffector endEffector) {
    this.endEffector = endEffector;
    this.currentState = null;
    this.targetState = null;
    this.currentTargetState = null;
    loadTransitionCosts();
    autoGenerateTransitions();
    precomputeAllPaths();
  }

  public void addTransition(SuperstructureTransition transition) {
    states.add(transition.getFromState());
    states.add(transition.getToState());
    transitions.add(transition);
  }

  public List<SuperstructureTransition> computeTransitionPaths(
      SuperstructureState from, SuperstructureState to) {
    graph = new HashMap<>();
    for (SuperstructureState state : states) {
      graph.put(state, new ArrayList<>());
    }
    for (SuperstructureTransition t : transitions) {
      if (!t.hasCollision()) {
        graph.get(t.getFromState()).add(t);
      }
    }
    AStarSolver<SuperstructureState> solver = new AStarSolver<>();
    List<SuperstructureState> statePath =
        solver.solve(
            from,
            to,
            (current, goal) -> current != null && current.equals(goal) ? 0 : 1,
            state -> {
              List<AStarSolver.Edge<SuperstructureState>> neighbors = new ArrayList<>();
              for (SuperstructureTransition t : graph.get(state)) {
                neighbors.add(new AStarSolver.Edge<>(t.getToState(), t.getTransitionTime()));
              }
              return neighbors;
            });
    if (statePath == null) return null;
    List<SuperstructureTransition> transitionPath = new ArrayList<>();
    for (int i = 0; i < statePath.size() - 1; i++) {
      SuperstructureState currentFrom = statePath.get(i);
      SuperstructureState currentTo = statePath.get(i + 1);
      Optional<SuperstructureTransition> transition =
          graph.get(currentFrom).stream().filter(t -> t.getToState().equals(currentTo)).findFirst();
      if (transition.isPresent()) {
        transitionPath.add(transition.get());
      } else {
        throw new IllegalStateException(
            "No transition found from " + currentFrom + " to " + currentTo);
      }
    }
    return transitionPath;
  }

  private List<SuperstructureTransition> getPrecomputedPath(
      SuperstructureState from, SuperstructureState to) {
    return precomputedPaths[from.ordinal()][to.ordinal()];
  }

  public List<SuperstructureTransition> computeDynamicTransitionPath(
      SuperstructureState from, SuperstructureState to) {
    Map<SuperstructureState, List<SuperstructureTransition>> dynamicGraph = new HashMap<>();
    for (SuperstructureState state : states) {
      dynamicGraph.put(state, new ArrayList<>());
    }
    for (SuperstructureTransition t : transitions) {
      if (!t.hasCollision() && !isTransitionBlocked(t)) {
        dynamicGraph.get(t.getFromState()).add(t);
      }
    }
    AStarSolver<SuperstructureState> solver = new AStarSolver<>();
    List<SuperstructureState> statePath =
        solver.solve(
            from,
            to,
            (current, goal) -> current.equals(goal) ? 0 : 1,
            state -> {
              List<AStarSolver.Edge<SuperstructureState>> neighbors = new ArrayList<>();
              for (SuperstructureTransition t : dynamicGraph.get(state)) {
                neighbors.add(new AStarSolver.Edge<>(t.getToState(), t.getTransitionTime()));
              }
              return neighbors;
            });
    if (statePath == null) return null;
    List<SuperstructureTransition> transitionPath = new ArrayList<>();
    for (int i = 0; i < statePath.size() - 1; i++) {
      SuperstructureState currentFrom = statePath.get(i);
      SuperstructureState currentTo = statePath.get(i + 1);
      Optional<SuperstructureTransition> transition =
          dynamicGraph.get(currentFrom).stream()
              .filter(t -> t.getToState().equals(currentTo))
              .findFirst();
      if (transition.isPresent()) {
        transitionPath.add(transition.get());
      } else {
        throw new IllegalStateException(
            "No dynamic transition found from " + currentFrom + " to " + currentTo);
      }
    }
    return transitionPath;
  }

  private boolean isTransitionBlocked(SuperstructureTransition transition) {
    SuperstructureState toState = transition.getToState();
    return toState.isCoralState() && RobotState.hasAlgae();
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }

  public SuperstructureState getCurrentTargetState() {
    return currentTargetState;
  }

  public void setCurrentState(SuperstructureState state) {
    if (!states.contains(state)) {
      throw new IllegalArgumentException("State not registered: " + state);
    }
    currentState = state;
  }

  public void setTargetState(SuperstructureState state) {
    if (!states.contains(state)) {
      throw new IllegalArgumentException("State not registered: " + state);
    }
    targetState = state;
  }

  private double currentTargetStateTime = 0;

  public void setCurrentTargetState(SuperstructureState state) {
    if (!states.contains(state)) {
      throw new IllegalArgumentException("State not registered: " + state);
    }
    currentTargetState = state;
    currentTargetStateTime = Timer.getFPGATimestamp();
  }

  public SuperstructureState getFutureDesiredState() {
    if (currentTargetState != null && Timer.getFPGATimestamp() - currentTargetStateTime > 3) {
      currentTargetState = null;
    }
    return currentTargetState;
  }

  public void setTargetState(SuperstructureState state, boolean setFuture, boolean wipeFuture) {
    if (currentState == null && targetState != null) {
      if (setFuture) setCurrentTargetState(state);
      return;
    }
    if (!states.contains(state)) {
      throw new IllegalArgumentException("State not registered: " + state);
    }
    targetState = state;
    if (!DriverStation.isAutonomous() && wipeFuture) currentTargetState = null;
    continueTransition();
  }

  public void continueTransition() {
    if (isTransitioning) {
      return;
    }

    if (currentState == null) {
      new ParallelCommandGroup(
              new InstantCommand(
                  () -> {
                    setCurrentState(targetState);
                    isTransitioning = false;
                    if (!currentState.equals(targetState)) {
                      continueTransition();
                    }
                  }))
          .withName("SuperstructureMove")
          .ignoringDisable(true)
          .schedule();
      return;
    }

    if (currentState.equals(targetState)) {
      return;
    }

    List<SuperstructureTransition> path = getPrecomputedPath(currentState, targetState);
    if (path == null || path.isEmpty()) {
      setTargetState(currentState);
      return;
    }

    SuperstructureTransition nextTransitionTemp = path.get(0);
    if (isTransitionBlocked(nextTransitionTemp)) {
      Logger.recordOutput(
          "Superstructure/BlockedTransition",
          "Precomputed transition "
              + nextTransitionTemp.toString()
              + " is blocked. Searching for alternative.");
      List<SuperstructureTransition> alternativePath =
          computeDynamicTransitionPath(currentState, targetState);
      if (alternativePath != null && !alternativePath.isEmpty()) {
        nextTransitionTemp = alternativePath.get(0);
      } else {
        Logger.recordOutput(
            "Superstructure/BlockedTransition",
            "No alternative transition available from " + currentState);
        return;
      }
    }
    final SuperstructureTransition nextTransition = nextTransitionTemp;
    isTransitioning = true;
    Command wrappedCommand =
        new ParallelCommandGroup(
            new InstantCommand(
                () -> {
                  setCurrentState(nextTransition.getToState());
                  isTransitioning = false;
                  if (!currentState.equals(targetState)) {
                    continueTransition();
                  }
                }));
    wrappedCommand.withName("SuperstructureMove").ignoringDisable(true).schedule();
  }
}
