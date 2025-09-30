package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  // private static Superstructure superstructureSubsystem;
  private EndEffector endEffector;
  private Elevator elevator;
  private SuperstructureStateMachine stateMachine;

  public Superstructure(Elevator elevator, EndEffector endEffector) {
    this.endEffector = endEffector;
    this.elevator = elevator;
    this.stateMachine = new SuperstructureStateMachine();
    Logger.recordOutput("Superstructure/SubsystemOnline", true);
  }

  @Override
  public void periodic() {
    stateMachine.continueTransition();
    Logger.recordOutput("Superstructure/CurrentState", stateMachine.getCurrentState());
    Logger.recordOutput("Superstructure/TargetState", stateMachine.getTargetState());
    Logger.recordOutput("Superstructure/CurrentTargetState", stateMachine.getCurrentTargetState());
    Logger.recordOutput("Superstructure/FutureDesiredState", stateMachine.getFutureDesiredState());
  }

  public Command setStateCommand(SuperstructureState state, String name) {
    return new InstantCommand(() -> stateMachine.setTargetState(state)).withName(name);
  }

  public Command setStateCommand(SuperstructureState state, boolean setFuture, String name) {
    return new InstantCommand(() -> stateMachine.setTargetState(state, setFuture, true))
        .withName(name);
  }

  public SuperstructureState getCurrentState() {
    return stateMachine.getCurrentState();
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
