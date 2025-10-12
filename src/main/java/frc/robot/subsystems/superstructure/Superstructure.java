package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.util.RobotTime;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  // private static Superstructure superstructureSubsystem;
  private EndEffector endEffector;
  private Elevator elevator;
  private SuperstructureStateMachine stateMachine;

  public Superstructure(Elevator elevator, EndEffector endEffector, RobotContainer container) {
    this.endEffector = endEffector;
    this.elevator = elevator;
    this.stateMachine = new SuperstructureStateMachine(container);
    Logger.recordOutput("Superstructure/SubsystemOnline", true);
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    stateMachine.continueTransition();
    RobotState.setSuperstructureState(getCurrentState());
    Logger.recordOutput("Superstructure/CurrentState", stateMachine.getCurrentState());
    Logger.recordOutput("Superstructure/TargetState", stateMachine.getTargetState());
    Logger.recordOutput("Superstructure/CurrentTargetState", stateMachine.getCurrentTargetState());
    Logger.recordOutput("Superstructure/FutureDesiredState", stateMachine.getFutureDesiredState());
    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    Logger.recordOutput("CoralStateTracker/Coral State", CoralStateTracker.getCurrentPosition());
  }

  public Command setStateCommand(SuperstructureState state, String name) {
    return Commands.either(
        clearCommandsIfManualOverride()
            .andThen(new InstantCommand(() -> stateMachine.setTargetState(state)).withName(name)),
        new InstantCommand(() -> stateMachine.setTargetState(state)).withName(name),
        () -> RobotState.getSuperstructureManualOverrideMode());
  }

  public Command setStateCommand(Supplier<SuperstructureState> stateSupplier, String name) {
    return Commands.either(
        clearCommandsIfManualOverride()
            .andThen(
                new InstantCommand(() -> stateMachine.setTargetState(stateSupplier.get()))
                    .withName(name)),
        new InstantCommand(() -> stateMachine.setTargetState(stateSupplier.get())).withName(name),
        () -> RobotState.getSuperstructureManualOverrideMode());
  }

  public Command setStateCommand(SuperstructureState state, boolean setFuture, String name) {
    return Commands.either(
        clearCommandsIfManualOverride()
            .andThen(
                new InstantCommand(() -> stateMachine.setTargetState(state, setFuture, true))
                    .withName(name)),
        new InstantCommand(() -> stateMachine.setTargetState(state, setFuture, true))
            .withName(name),
        () -> RobotState.getSuperstructureManualOverrideMode());
  }

  public Command clearCommandsIfManualOverride() {
    return Commands.runOnce(() -> elevator.getCurrentCommand().cancel())
        .asProxy()
        .onlyIf(() -> elevator.getCurrentCommand() != null)
        .alongWith(
            Commands.runOnce(() -> endEffector.getCurrentCommand().cancel())
                .asProxy()
                .onlyIf(() -> endEffector.getCurrentCommand() != null))
        .asProxy()
        .alongWith(
            Commands.print("    WARNING: HARD CLEAR SUPERSTRUCTURE COMMANDS")
                .alongWith(Commands.runOnce(() -> stateMachine.hardSetIsTransitioning(false))));
  }

  public SuperstructureState getCurrentState() {
    return stateMachine.getCurrentState();
  }

  public SuperstructureState getCurrentTargetState() {
    return stateMachine.getCurrentTargetState();
  }

  public double calculateDynamicTranslationalAccelLimit() {

    // Get subsystem positions
    double elevatorHeight = elevator.getCurrentPosition(); // inches
    double endEffectorPivotPosition = endEffector.getCurrentPivotPosition(); // radians

    //  E- elevator.height*b - intakePivot.height*c-(endEffectorPivot.height*a+elevator.height)*d
    double dynamicLimit =
        Constants.DriveConstants.MAX_TRANSLATIONAL_ACCEL
            - elevatorHeight * Constants.DriveConstants.DYNAMIC_ACCEL_WEIGHT_B
            - (endEffectorPivotPosition * Constants.DriveConstants.DYNAMIC_ACCEL_WEIGHT_A
                    + elevatorHeight)
                * Constants.DriveConstants.DYNAMIC_ACCEL_WEIGHT_D;

    // Log individual components for debugging
    Logger.recordOutput("Superstructure/ElevatorHeight", elevatorHeight);
    Logger.recordOutput("Superstructure/EndEffectorPivotPosition", endEffectorPivotPosition);
    Logger.recordOutput("Superstructure/DynamicTranslationalLimit", dynamicLimit);

    return dynamicLimit;
  }
  // Uses the same formula as translational for now, but could be different in the future.
  public double calculateDynamicRotationalAccelLimit() {
    // Get subsystem positions
    double elevatorHeight = elevator.getCurrentPosition(); // inches
    double endEffectorPivotPosition = endEffector.getCurrentPivotPosition(); // radians

    //  E  - elevator.height*b - intakePivot.height*c-(endEffectorPivot.height*a+elevator.height)*d
    double dynamicLimit =
        Constants.DriveConstants.MAX_ROTATIONAL_ACCEL
            - elevatorHeight * Constants.DriveConstants.DYNAMIC_ACCEL_WEIGHT_B
            - (endEffectorPivotPosition * Constants.DriveConstants.DYNAMIC_ACCEL_WEIGHT_A
                    + elevatorHeight)
                * Constants.DriveConstants.DYNAMIC_ACCEL_WEIGHT_D;

    Logger.recordOutput("Superstructure/DynamicRotationalLimit", dynamicLimit);

    return dynamicLimit;
  }

  public void setTriggers() {}
}
