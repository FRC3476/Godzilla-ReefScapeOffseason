package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;

public class ElevatorEndEffectorTest extends SequentialCommandGroup {
  /** Creates a new ElevatorEndEffectorTest. */
  public ElevatorEndEffectorTest(Elevator elevator, EndEffector endEffector) {
    addCommands(

        endEffector.rollerFWD(),
        new WaitCommand(2.0),

        endEffector.rollerSTOP(),
        new WaitCommand(0.5),

        endEffector.rollerRVS(),
        new WaitCommand(2.0),

        endEffector.rollerSTOP(),
        new WaitCommand(0.5),

        elevator.elevatorUP(),
        new WaitCommand(1.5),

        elevator.elevatorSTOP(),
        new WaitCommand(0.5),

        elevator.elevatorDWN(),
        new WaitCommand(1.5),

        elevator.elevatorSTOP(),
        new WaitCommand(0.5),

        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_L2_SETPOINT_INCH),
        new WaitCommand(2.0),

        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_L3_SETPOINT_INCH),
        new WaitCommand(2.0),

        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH),
        new WaitCommand(2.0),

        endEffector.moveToTargetRadian(Math.toRadians(45.0)),
        new WaitCommand(1.5),

        endEffector.moveToTargetRadian(Math.toRadians(90.0)),
        new WaitCommand(1.5),

        endEffector.moveToTargetRadian(0.0),
        new WaitCommand(1.5),

        elevator.elevatorSTOP(),
        endEffector.rollerSTOP()
    );
  }
}