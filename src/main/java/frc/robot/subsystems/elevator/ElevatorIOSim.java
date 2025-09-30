package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim extends ElevatorIOReal {

  protected ElevatorSim elevatorSim;
  protected Notifier simNotifier;
  private final TalonFXSimState rightSimState;
  private final TalonFXSimState leftSimState;
  private final TalonFXSimState extraSimState;
  protected double lastUpdateTimestamp = 0.0;

  public ElevatorIOSim() {
    super();

    // Initialize ElevatorSim with appropriate parameters
    elevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(3), // Three Falcon 500 motors
            1.0 / ElevatorConstants.kGearing,
            ElevatorConstants.CARRIAGE_MASS_KG,
            ElevatorConstants.DRUM_RADIUS_METERS,
            ElevatorConstants.MIN_HEIGHT_METERS,
            ElevatorConstants.MAX_HEIGHT_METERS,
            true // Simulate gravity
            ,
            0);

    // Access the simulation state of the TalonFX motors
    rightSimState = rightTalon.getSimState();
    leftSimState = leftTalon.getSimState();
    extraSimState = extraTalon.getSimState();

    rightSimState.Orientation = ChassisReference.Clockwise_Positive;
    leftSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    extraSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    // Set up a Notifier to periodically update the simulation
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  private void updateSimState() {
    // Set Supply Voltage
    rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get the applied voltage from the right motor (leader)
    double appliedVoltage = rightSimState.getMotorVoltage();

    // Apply the motor voltage to the simulation
    elevatorSim.setInputVoltage(appliedVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // Update the simulation by 5 ms
    double timestamp = Timer.getFPGATimestamp();
    elevatorSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    // Convert simulation output to motor rotations
    double positionMeters = elevatorSim.getPositionMeters();
    double velocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    double currentAmps = elevatorSim.getCurrentDrawAmps();

    // Convert to rotations for TalonFX simulation
    double positionRotations = positionMeters / ElevatorConstants.kElevatorUnitToRotorRatio;
    double velocityRPS = velocityMetersPerSecond / ElevatorConstants.kElevatorUnitToRotorRatio;

    // Update the simulation states for all motors
    rightSimState.setRawRotorPosition(positionRotations);
    rightSimState.setRotorVelocity(velocityRPS);
    leftSimState.setRawRotorPosition(positionRotations);
    leftSimState.setRotorVelocity(velocityRPS);
    extraSimState.setRawRotorPosition(positionRotations);
    extraSimState.setRotorVelocity(velocityRPS);

    // Log simulation data for debugging and visualization
    Logger.recordOutput("Elevator/Sim/PositionMeters", positionMeters);
    Logger.recordOutput("Elevator/Sim/VelocityMetersPerSecond", velocityMetersPerSecond);
    Logger.recordOutput("Elevator/Sim/CurrentAmps", currentAmps);
    Logger.recordOutput("Elevator/Sim/AppliedVoltage", appliedVoltage);
    Logger.recordOutput("Elevator/Sim/PositionRotations", positionRotations);
    Logger.recordOutput("Elevator/Sim/VelocityRPS", velocityRPS);
    Logger.recordOutput("Elevator/Sim/RightMotorVoltage", rightSimState.getMotorVoltage());
    Logger.recordOutput("Elevator/Sim/LeftMotorVoltage", leftSimState.getMotorVoltage());
    Logger.recordOutput("Elevator/Sim/ExtraMotorVoltage", extraSimState.getMotorVoltage());
  }

  // Clean up resources when simulation ends
  public void close() {
    simNotifier.close();
  }
}
