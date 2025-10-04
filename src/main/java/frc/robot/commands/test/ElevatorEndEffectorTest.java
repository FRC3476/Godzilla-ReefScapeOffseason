package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;

public class ElevatorEndEffectorTest extends SequentialCommandGroup {

  private static final double POSITION_TOLERANCE_RAD = Math.toRadians(5.0);
  private static final double MOVEMENT_TIMEOUT_SECONDS = 5.0;

  /** Creates a new ElevatorEndEffectorTest */
  public ElevatorEndEffectorTest(Elevator elevator, EndEffector endEffector) {
    addCommands(
        Commands.print("Starting Elevator & End Effector Automated Test"),
        Commands.print("Elevator Homing"),
        elevator.homeElevator(),
        new WaitCommand(1.0),
        Commands.print("Elevator homing completed"),
        Commands.print("Moving to Safe Position"),
        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH),
        new WaitUntilCommand(() -> elevator.isInTolerance()).withTimeout(MOVEMENT_TIMEOUT_SECONDS),
        Commands.print("Elevator at safe position (0 inches)"),
        endEffector.moveToTargetRadian(EndEffectorConstants.IDLE_ANGLE_RAD),
        new WaitUntilCommand(
                () ->
                    endEffector.isInTolerance(
                        EndEffectorConstants.IDLE_ANGLE_RAD, POSITION_TOLERANCE_RAD))
            .withTimeout(MOVEMENT_TIMEOUT_SECONDS),
        Commands.print("End effector at safe idle position"),
        Commands.print("End Effector Testing"),
        Commands.print("Testing end effector rollers..."),
        endEffector.rollerFWD(),
        new WaitCommand(2.0),
        endEffector.rollerSTOP(),
        new WaitCommand(0.5),
        endEffector.rollerRVS(),
        new WaitCommand(2.0),
        endEffector.rollerSTOP(),
        new WaitCommand(0.5),
        Commands.print("Roller testing completed"),
        Commands.print("Testing end effector pivot movement..."),
        createSafeEndEffectorMovement(endEffector, EndEffectorConstants.ALGAE_IDLE_ANGLE_RAD),
        new WaitCommand(1.5),
        createSafeEndEffectorMovement(endEffector, EndEffectorConstants.GROUND_ALGAE_ANGLE_RAD),
        new WaitCommand(1.5),
        createSafeEndEffectorMovement(endEffector, EndEffectorConstants.PROCESSOR_ANGLE_RAD),
        new WaitCommand(1.5),
        createSafeEndEffectorMovement(endEffector, EndEffectorConstants.IDLE_ANGLE_RAD),
        new WaitCommand(1.0),
        Commands.print("End effector pivot testing completed"),
        Commands.print("Elevator Testing (Position-based)"),
        Commands.print("Testing elevator L2 position..."),
        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_L2_SETPOINT_INCH),
        new WaitUntilCommand(() -> elevator.isInTolerance()).withTimeout(MOVEMENT_TIMEOUT_SECONDS),
        new WaitCommand(2.0),
        Commands.print("Testing elevator L3 position..."),
        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_L3_SETPOINT_INCH),
        new WaitUntilCommand(() -> elevator.isInTolerance()).withTimeout(MOVEMENT_TIMEOUT_SECONDS),
        new WaitCommand(2.0),
        Commands.print("Testing elevator intake position..."),
        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_INTAKE_SETPOINT_INCH),
        new WaitUntilCommand(() -> elevator.isInTolerance()).withTimeout(MOVEMENT_TIMEOUT_SECONDS),
        new WaitCommand(2.0),
        Commands.print("Phase 5: Returning to Safe State"),
        elevator.moveToTargetPosition(ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH),
        new WaitUntilCommand(() -> elevator.isInTolerance()).withTimeout(MOVEMENT_TIMEOUT_SECONDS),
        createSafeEndEffectorMovement(endEffector, EndEffectorConstants.IDLE_ANGLE_RAD),
        new WaitUntilCommand(
                () ->
                    endEffector.isInTolerance(
                        EndEffectorConstants.IDLE_ANGLE_RAD, POSITION_TOLERANCE_RAD))
            .withTimeout(MOVEMENT_TIMEOUT_SECONDS),
        elevator.elevatorSTOP(),
        endEffector.rollerSTOP(),
        Commands.print("Elevator & End Effector Automated Test COMPLETED SUCCESSFULLY"));
  }

  /** Creates a safe end effector movement command with built-in safety checks. */
  private Command createSafeEndEffectorMovement(EndEffector endEffector, double targetAngleRad) {
    return Commands.either(
        Commands.sequence(
            Commands.print("Moving end effector to " + Math.toDegrees(targetAngleRad) + " degrees"),
            endEffector.moveToTargetRadian(targetAngleRad),
            new WaitUntilCommand(
                    () -> endEffector.isInTolerance(targetAngleRad, POSITION_TOLERANCE_RAD))
                .withTimeout(MOVEMENT_TIMEOUT_SECONDS)),
        Commands.print(
            "Skipping unsafe EndEffector movement because "
                + Math.toDegrees(targetAngleRad)
                + " degrees"),
        () ->
            targetAngleRad >= EndEffectorConstants.SAFE_ANGLE_LOWER_RAD
                && targetAngleRad <= EndEffectorConstants.SAFE_ANGLE_UPPER_RAD);
  }
}
