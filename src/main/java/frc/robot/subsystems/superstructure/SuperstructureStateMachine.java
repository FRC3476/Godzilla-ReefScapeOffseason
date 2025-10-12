package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
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

/*
Pompt
I am working inside of SuperstructureStateMachine

I want to make this code as easy to maintain and read as possible.
Without changing the core functionality propose changes that can make this code easier to read and test. You do not need to write any test functions.

I'll analyze the SuperstructureStateMachine code to understand its current structure and propose improvements for better maintainability and readability. Let me start by examining the file.
Now let me examine the related classes to better understand the structure and dependencies:
Let me also check the AStarSolver class to understand the pathfinding implementation:
Now I have a comprehensive understanding of the SuperstructureStateMachine class. Based on my analysis, here are the key areas for improvement to make the code more maintainable and readable:
Proposed Improvements for SuperstructureStateMachine
1. Extract Constants and Magic Numbers
Extract magic numbers like 3 (timeout), 1.0 (default cost), and 1 (heuristic value)
Create named constants for better readability
2. Break Down Large Methods
The continueTransition() method is doing too much and should be split
The path computation methods have duplicate code that can be extracted
3. Improve Method Naming and Documentation
Add comprehensive JavaDoc comments
Use more descriptive method names
Add inline comments for complex logic
4. Extract Pathfinding Logic
Create a separate Pathfinder class to handle A* pathfinding
Reduce duplication between computeTransitionPaths and computeDynamicTransitionPath
5. Improve State Management
Create a StateManager inner class to handle state-related operations
Better encapsulate state validation logic
6. Extract Command Creation Logic
Create a CommandFactory to handle command creation and scheduling
Reduce the complexity of transition execution
7. Add Input Validation and Error Handling
Better error messages and validation
More robust handling of edge cases
8. Improve Code Organization
Group related methods together
Use consistent formatting and spacing
Here's the refactored code with these improvements:
Key Improvements Made:
Constants: Extracted all magic numbers into named constants at the top of the class
Method Decomposition: Broke down large methods into smaller, focused methods
Inner Classes: Created StateManager, Pathfinder, and CommandFactory to separate concerns
Documentation: Added comprehensive JavaDoc comments for all public methods
Error Handling: Improved validation and error messages
Code Organization: Grouped related methods together with clear section comments
Reduced Duplication: Eliminated duplicate pathfinding code by extracting common logic
Better Naming: Used more descriptive method and variable names
Separation of Concerns: Each inner class has a single responsibility
Improved Readability: Better formatting and consistent code style
These changes maintain the exact same functionality while making the code much more maintainable, testable, and readable.
Here's the refactored code with these improvements: */

/**
 * Manages state transitions for the superstructure using A* pathfinding. This class calculates
 * optimal paths between states and handles dynamic re-routing when transitions are blocked.
 */
public class SuperstructureStateMachine {

  // Constants
  private static final double DEFAULT_TRANSITION_COST = 1.0;
  private static final double FUTURE_STATE_TIMEOUT_SECONDS = 3.0;
  private static final String TRANSITION_COSTS_FILE = "transition_costs.txt";
  private static final String TRANSITION_KEY_SEPARATOR = "->";
  private static final String COMMAND_NAME = "SuperstructureMove";

  // Core data structures
  private final Set<SuperstructureState> registeredStates = new HashSet<>();
  private final List<SuperstructureTransition> transitions = new ArrayList<>();
  private final Map<String, Double> transitionCostMap = new HashMap<>();

  // State management
  private final StateManager stateManager = new StateManager();

  // Pathfinding
  private final Pathfinder pathfinder = new Pathfinder();

  // Command execution
  private final CommandFactory commandFactory = new CommandFactory();

  // Precomputed paths for performance
  private List<SuperstructureTransition>[][] precomputedPaths;
  private boolean isTransitioning = false;

  // Container thingy
  private RobotContainer container;

  /** Constructs a new SuperstructureStateMachine. */
  public SuperstructureStateMachine(RobotContainer container) {
    this.container = container;
    initializeStateMachine();
    stateManager
        .getCurrentTargetState()
        .getCommand(container)
        .schedule(); // sets the intial setpoints to stow
  }

  /**
   * Initializes the state machine by loading costs, generating transitions, and precomputing paths.
   */
  private void initializeStateMachine() {
    loadTransitionCosts();
    autoGenerateTransitions();
    precomputeAllPaths();
  }

  public void hardSetIsTransitioning(boolean b) {
    isTransitioning = b;
  }

  // ==================== PUBLIC API ====================

  /**
   * Gets the current state of the superstructure.
   *
   * @return The current state, or null if not set
   */
  public SuperstructureState getCurrentState() {
    return stateManager.getCurrentState();
  }

  /**
   * Gets the target state the superstructure is trying to reach.
   *
   * @return The target state, or null if not set
   */
  public SuperstructureState getTargetState() {
    return stateManager.getTargetState();
  }

  /**
   * Gets the immediate next state in the transition path.
   *
   * @return The current target state, or null if not set
   */
  public SuperstructureState getCurrentTargetState() {
    return stateManager.getCurrentTargetState();
  }

  /**
   * Sets the current state of the superstructure.
   *
   * @param state The new current state
   * @throws IllegalArgumentException if the state is not registered
   */
  public void setCurrentState(SuperstructureState state) {
    stateManager.setCurrentState(state, registeredStates);
  }

  /**
   * Sets the target state for the superstructure.
   *
   * @param state The new target state
   * @throws IllegalArgumentException if the state is not registered
   */
  public void setTargetState(SuperstructureState state) {
    if (RobotState.getSuperstructureManualOverrideMode()) {
      setTargetState(state, true, true);
    } else {
      stateManager.setTargetState(state, registeredStates);
      continueTransition();
    }
  }

  /**
   * Sets the target state with additional options for future state management.
   *
   * @param state The new target state
   * @param setFuture Whether to set this as a future desired state
   * @param wipeFuture Whether to clear future states in teleop
   */
  public void setTargetState(SuperstructureState state, boolean setFuture, boolean wipeFuture) {
    if (stateManager.getCurrentState() == null && stateManager.getTargetState() != null) {
      if (setFuture) {
        stateManager.setCurrentTargetState(state, registeredStates);
      }
      return;
    }

    stateManager.setTargetState(state, registeredStates);

    if (!DriverStation.isAutonomous() && wipeFuture) {
      stateManager.forceSetCurrentTargetState(state);
    }

    continueTransition();
  }

  /**
   * Gets the future desired state if it hasn't timed out.
   *
   * @return The future desired state, or null if none or timed out
   */
  public SuperstructureState getFutureDesiredState() {
    return stateManager.getFutureDesiredState();
  }

  /**
   * Gets the transition cost between two states.
   *
   * @param from The source state
   * @param to The destination state
   * @return The transition cost, or default cost if not specified
   */
  public double getTransitionCost(SuperstructureState from, SuperstructureState to) {
    return transitionCostMap.getOrDefault(
        getTransitionKey(from, to),
        DEFAULT_TRANSITION_COST
            + Math.abs(to.getEndEffectorRotation() - from.getEndEffectorRotation()));
  }

  /**
   * Continues the transition process to reach the target state. This method handles the main
   * transition logic and command scheduling.
   */
  public void continueTransition() {
    Logger.recordOutput("Superstructure/IsTransitioning", isTransitioning);
    if (isTransitioning) {
      return;
    }

    SuperstructureState currentState = stateManager.getCurrentState();
    SuperstructureState targetState = stateManager.getTargetState();

    // Handle initial state setup
    if (currentState == null) {
      handleInitialStateTransition(targetState);
      return;
    }

    // Check if already at target
    if (currentState.equals(targetState)) {
      return;
    }

    // Find and execute next transition
    executeNextTransition(currentState, targetState);
  }

  // ==================== PRIVATE METHODS ====================

  /** Handles the initial state transition when current state is null. */
  private void handleInitialStateTransition(SuperstructureState targetState) {

    stateManager.setCurrentTargetState(targetState, registeredStates);

    Command checkFinishedCommand =
        commandFactory.createStateTransitionCommand(
            () -> {
              stateManager.setCurrentState(targetState, registeredStates);
              isTransitioning = false;
              if (!stateManager.getCurrentState().equals(stateManager.getTargetState())) {
                continueTransition();
              }
            });
    Command command =
        Commands.sequence(
                stateManager.getCurrentTargetState().getCommand(container), checkFinishedCommand)
            .withName(stateManager.getCurrentTargetState().name() + "_StateMachineInitial");
    command.schedule();
  }

  /** Executes the next transition in the path to the target state. */
  private void executeNextTransition(
      SuperstructureState currentState, SuperstructureState targetState) {

    List<SuperstructureTransition> path = getPrecomputedPath(currentState, targetState);

    if (path == null || path.isEmpty()) {
      // No path available - set target to current state to effectively cancel the transition
      stateManager.setTargetState(currentState, registeredStates);
      return;
    }

    SuperstructureTransition nextTransition = findValidTransition(path, currentState, targetState);
    if (nextTransition == null) {
      return; // No valid transition found
    }

    executeTransition(nextTransition);
  }

  /** Finds a valid transition from the given path, using dynamic pathfinding if needed. */
  private SuperstructureTransition findValidTransition(
      List<SuperstructureTransition> path,
      SuperstructureState currentState,
      SuperstructureState targetState) {

    SuperstructureTransition nextTransition = path.get(0);

    if (!isTransitionBlocked(nextTransition)) {
      return nextTransition;
    }

    // Try dynamic pathfinding for blocked transitions
    Logger.recordOutput(
        "Superstructure/BlockedTransition",
        "Precomputed transition "
            + nextTransition.toString()
            + " is blocked. Searching for alternative.");

    List<SuperstructureTransition> alternativePath =
        pathfinder.computeDynamicTransitionPath(
            currentState, targetState, transitions, registeredStates, this::isTransitionBlocked);

    if (alternativePath != null && !alternativePath.isEmpty()) {
      return alternativePath.get(0);

    } else {
      Logger.recordOutput("Superstructure/BlockedTransition", true);
      return null;
    }
  }

  /** Executes a transition by scheduling the appropriate command. */
  private void executeTransition(SuperstructureTransition transition) {

    Logger.recordOutput("Superstructure/Transition", transition.toString());
    isTransitioning = true;

    stateManager.setCurrentTargetState(transition.getToState(), registeredStates);

    Command checkFinishedCommand =
        commandFactory.createStateTransitionCommand(
            () -> {
              stateManager.setCurrentState(transition.getToState(), registeredStates);
              isTransitioning = false;
              if (!stateManager.getCurrentState().equals(stateManager.getTargetState())) {
                continueTransition();
              }
            });
    Command moveCommand =
        stateManager.getCurrentState().equals(stateManager.getTargetState())
            ? stateManager.getCurrentTargetState().getCommand(container)
                : stateManager.getCurrentTargetState().getAsTransitionCommand(container, getCurrentState());

    Command command = Commands.sequence(moveCommand, checkFinishedCommand)
            .withName(stateManager.getCurrentTargetState().name() + "_StateMachineExecute");
    command.schedule();
  }

  /** Checks if a transition is blocked by current conditions. */
  private boolean isTransitionBlocked(SuperstructureTransition transition) {
    SuperstructureState toState = transition.getToState();
    SuperstructureState fromState = transition.getFromState();
    return toState.isCoralState() && RobotState.hasAlgae();
  }

  /** Gets a precomputed path between two states. */
  private List<SuperstructureTransition> getPrecomputedPath(
      SuperstructureState from, SuperstructureState to) {
    return precomputedPaths[from.ordinal()][to.ordinal()];
  }

  /** Generates a transition key for cost lookup. */
  private String getTransitionKey(SuperstructureState from, SuperstructureState to) {
    return from.name() + TRANSITION_KEY_SEPARATOR + to.name();
  }

  // ==================== INITIALIZATION METHODS ====================

  /** Loads transition costs from the configuration file. */
  private void loadTransitionCosts() {
    transitionCostMap.clear();
    File costFile = new File(Filesystem.getDeployDirectory(), TRANSITION_COSTS_FILE);

    try (BufferedReader br = new BufferedReader(new FileReader(costFile))) {
      String line;
      while ((line = br.readLine()) != null) {
        parseTransitionCostLine(line);
      }
      Logger.recordOutput("Using Transition Costs", true);
    } catch (IOException e) {
      Logger.recordOutput(
          "Superstructure/Error", "Failed to load transition costs: " + e.getMessage());
      Logger.recordOutput("Using Transition Costs", false);
    }
  }

  /** Parses a single line from the transition costs file. */
  private void parseTransitionCostLine(String line) {
    String[] parts = line.split(",");
    if (parts.length == 3) {
      String key = parts[0] + TRANSITION_KEY_SEPARATOR + parts[1];
      double duration = Double.parseDouble(parts[2]);
      transitionCostMap.put(key, duration);
    }
  }

  /** Automatically generates transitions between all allowed states. */
  public void autoGenerateTransitions() {
    for (SuperstructureState from : SuperstructureState.values()) {
      for (SuperstructureState to : from.getAllowedDestinationStates()) {
        double cost = getTransitionCost(from, to);
        addTransition(new SuperstructureTransition(from, to, cost, false));
      }
    }
  }

  /** Adds a transition to the state machine. */
  public void addTransition(SuperstructureTransition transition) {
    registeredStates.add(transition.getFromState());
    registeredStates.add(transition.getToState());
    transitions.add(transition);
  }

  /** Precomputes all possible paths between states for performance. */
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
              pathfinder.computeTransitionPaths(from, to, transitions, registeredStates);
        }
      }
    }
  }

  // ==================== INNER CLASSES ====================

  /** Manages state-related operations and validation. */
  private static class StateManager {
    private SuperstructureState currentState;
    private SuperstructureState targetState = SuperstructureState.NONE;
    private SuperstructureState currentTargetState = targetState;
    private double currentTargetStateTime = 0;

    public SuperstructureState getCurrentState() {
      return currentState;
    }

    public SuperstructureState getTargetState() {
      return targetState;
    }

    public SuperstructureState getCurrentTargetState() {
      return currentTargetState;
    }

    public void setCurrentState(SuperstructureState state, Set<SuperstructureState> validStates) {
      validateState(state, validStates);
      currentState = state;
    }

    public void setTargetState(SuperstructureState state, Set<SuperstructureState> validStates) {
      validateState(state, validStates);
      targetState = state;
    }

    public void setCurrentTargetState(
        SuperstructureState state, Set<SuperstructureState> validStates) {
      validateState(state, validStates);
      currentTargetState = state;
      currentTargetStateTime = Timer.getFPGATimestamp();
    }

    public void forceSetCurrentTargetState(SuperstructureState state) {
      currentTargetState = state;
    }

    public SuperstructureState getFutureDesiredState() {
      if (currentTargetState != null
          && Timer.getFPGATimestamp() - currentTargetStateTime > FUTURE_STATE_TIMEOUT_SECONDS) {
        currentTargetState = null;
      }
      return currentTargetState;
    }

    private void validateState(SuperstructureState state, Set<SuperstructureState> validStates) {
      if (!validStates.contains(state)) {
        throw new IllegalArgumentException("State not registered: " + state);
      }
    }
  }

  /** Handles pathfinding operations using A* algorithm. */
  private static class Pathfinder {

    /** Computes transition paths between two states using A* pathfinding. */
    public List<SuperstructureTransition> computeTransitionPaths(
        SuperstructureState from,
        SuperstructureState to,
        List<SuperstructureTransition> transitions,
        Set<SuperstructureState> registeredStates) {

      Map<SuperstructureState, List<SuperstructureTransition>> graph =
          buildGraph(transitions, registeredStates, true, transition -> false);
      return findPath(from, to, graph);
    }

    /** Computes dynamic transition paths considering current blocking conditions. */
    public List<SuperstructureTransition> computeDynamicTransitionPath(
        SuperstructureState from,
        SuperstructureState to,
        List<SuperstructureTransition> transitions,
        Set<SuperstructureState> registeredStates,
        java.util.function.Predicate<SuperstructureTransition> isBlocked) {

      Map<SuperstructureState, List<SuperstructureTransition>> graph =
          buildGraph(transitions, registeredStates, true, isBlocked);
      return findPath(from, to, graph);
    }

    /** Builds a graph from transitions for pathfinding with blocking check. */
    private Map<SuperstructureState, List<SuperstructureTransition>> buildGraph(
        List<SuperstructureTransition> transitions,
        Set<SuperstructureState> registeredStates,
        boolean checkCollisions,
        java.util.function.Predicate<SuperstructureTransition> isBlocked) {

      Map<SuperstructureState, List<SuperstructureTransition>> graph = new HashMap<>();

      for (SuperstructureState state : registeredStates) {
        graph.put(state, new ArrayList<>());
      }

      for (SuperstructureTransition transition : transitions) {
        if (!checkCollisions || (!transition.hasCollision() && !isBlocked.test(transition))) {
          graph.get(transition.getFromState()).add(transition);
        }
      }

      return graph;
    }

    /** Finds a path between two states using A* algorithm. */
    private List<SuperstructureTransition> findPath(
        SuperstructureState from,
        SuperstructureState to,
        Map<SuperstructureState, List<SuperstructureTransition>> graph) {

      AStarSolver<SuperstructureState> solver = new AStarSolver<>();
      List<SuperstructureState> statePath =
          solver.solve(
              from,
              to,
              (current, goal) -> current != null && current.equals(goal) ? 0 : 1,
              state -> {
                List<AStarSolver.Edge<SuperstructureState>> neighbors = new ArrayList<>();
                for (SuperstructureTransition transition : graph.get(state)) {
                  neighbors.add(
                      new AStarSolver.Edge<>(
                          transition.getToState(), transition.getTransitionTime()));
                }
                return neighbors;
              });

      if (statePath == null) {
        return null;
      }

      return convertStatePathToTransitionPath(statePath, graph);
    }

    /** Converts a state path to a transition path. */
    private List<SuperstructureTransition> convertStatePathToTransitionPath(
        List<SuperstructureState> statePath,
        Map<SuperstructureState, List<SuperstructureTransition>> graph) {

      List<SuperstructureTransition> transitionPath = new ArrayList<>();

      for (int i = 0; i < statePath.size() - 1; i++) {
        SuperstructureState currentFrom = statePath.get(i);
        SuperstructureState currentTo = statePath.get(i + 1);

        Optional<SuperstructureTransition> transition =
            graph.get(currentFrom).stream()
                .filter(t -> t.getToState().equals(currentTo))
                .findFirst();

        if (transition.isPresent()) {
          transitionPath.add(transition.get());
        } else {
          throw new IllegalStateException(
              "No transition found from " + currentFrom + " to " + currentTo);
        }
      }

      return transitionPath;
    }
  }

  /** Factory for creating and configuring commands. */
  private static class CommandFactory {

    /** Creates a command for state transitions. */
    public Command createStateTransitionCommand(Runnable action) {
      return new ParallelCommandGroup(new InstantCommand(action))
          .withName(COMMAND_NAME)
          .ignoringDisable(true);
    }
  }
}
