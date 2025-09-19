package frc.robot.subsystems.superstructure;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class Superstructure extends SubsystemBase{


    private final Elevator elevator;
    private final EndEffector endEffector;
    private final Intake intake;
    private final Feeder feeder;
    private SuperstructureStateMachine stateMachine;
    private CoralTracker coralTracker;

    public Superstructure(){
        this.elevator = Elevator.getInstance();
        this.endEffector = EndEffector.getInstance();
        this.intake = Intake.getInstance();
        this.feeder = Feeder.getInstance();
        this.coralTracker = new CoralTracker(intake, feeder, endEffector);
    }

    @Override
    public void periodic(){
        coralTracker.updateCoalPos();
    }

        private Command setStateCommand(SuperstructureState state, String name) {
        return new InstantCommand(() -> stateMachine.setTargetState(state))
                .withName(name);
    }

    private Command setStateCommand(SuperstructureState state, boolean setFuture, String name) {
        return new InstantCommand(() -> stateMachine.setTargetState(state, setFuture, true))
                .withName(name);
    }

    private Command updateStatePosition(){
        return new ParallelCommandGroup(
            elevator.moveToTargetPosition(stateMachine.getCurrentState().getElevatorHeight()),
            endEffector.rotatePivot(stateMachine.getCurrentState().getEndEffectorRotation())
        );
    }

    public void setTriggers(){
        
    }


    
}
