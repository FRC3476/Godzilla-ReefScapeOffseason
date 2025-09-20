package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.IntakeConstants;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim extends IntakeIOReal {

  // Simulation models for each subsystem
  protected DCMotorSim rollerSim;
  protected DCMotorSim pivotSim;
  protected DCMotorSim blockerSim;
  protected Notifier simNotifier;

  // TalonFX simulation states
  private final TalonFXSimState pivotSimState;
  private final TalonFXSimState rollerSimState;
  private final TalonFXSimState blockerSimState;

  // Simulation state tracking
  protected double lastUpdateTimestamp = 0.0;

  protected AtomicReference<Double> lastRotations = new AtomicReference<>((double) 0.0);
  protected AtomicReference<Double> lastRPS = new AtomicReference<>((double) 0.0);

  public IntakeIOSim() {
    super();

    // Initialize simulation models for each subsystem
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                IntakeConstants.ROLLER_MOI,
                1.0 / IntakeConstants.ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                IntakeConstants.PIVOT_MOI,
                1.0 / IntakeConstants.PIVOT_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    blockerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                IntakeConstants.L1_BAR_MOI,
                1.0 / IntakeConstants.L1_BAR_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    // Set up TalonFX simulation states
    pivotMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    rollerMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    lvl1blockerMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;

    pivotSimState = pivotMotor.getSimState();
    rollerSimState = rollerMotor.getSimState();
    blockerSimState = lvl1blockerMotor.getSimState();

    // Set up Notifier for periodic simulation updates
    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005); // 5ms update rate for better PID behavior
  }

  private void updateSimState() {
    // Set supply voltage for all motors
    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    blockerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get applied voltages from motors
    double pivotVoltage = pivotSimState.getMotorVoltage();
    double rollerVoltage = rollerSimState.getMotorVoltage();
    double blockerVoltage = blockerSimState.getMotorVoltage();

    // Apply voltages to simulation models
    pivotSim.setInputVoltage(pivotVoltage);
    rollerSim.setInputVoltage(rollerVoltage);
    blockerSim.setInputVoltage(blockerVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(blockerSim.getCurrentDrawAmps()));

    // Update simulation models
    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    pivotSim.update(dt);
    rollerSim.update(dt);
    blockerSim.update(dt);

    // Convert simulation output to motor rotations and update TalonFX simulation states
    updateMotorSimStates();

    // Log simulation data
    logSimulationData();
  }

  private void updateMotorSimStates() {
    updatePivotSimStates();
    updateRollerSimStates();
    updateL1BlockerSimStates();
  }

  private void logSimulationData() {
    // Log pivot simulation data
    Logger.recordOutput("Intake/Sim/Pivot/PositionRad", pivotSim.getAngularPositionRad());
    Logger.recordOutput("Intake/Sim/Pivot/VelocityRPS", pivotSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Intake/Sim/Pivot/CurrentAmps", pivotSim.getCurrentDrawAmps());
    Logger.recordOutput("Intake/Sim/Pivot/AppliedVoltage", pivotSimState.getMotorVoltage());

    // Log roller simulation data
    Logger.recordOutput("Intake/Sim/Roller/PositionRad", rollerSim.getAngularPositionRad());
    Logger.recordOutput("Intake/Sim/Roller/VelocityRPS", rollerSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Intake/Sim/Roller/CurrentAmps", rollerSim.getCurrentDrawAmps());
    Logger.recordOutput("Intake/Sim/Roller/AppliedVoltage", rollerSimState.getMotorVoltage());

    // Log blocker simulation data
    Logger.recordOutput("Intake/Sim/Blocker/PositionRad", blockerSim.getAngularPositionRad());
    Logger.recordOutput("Intake/Sim/Blocker/VelocityRPS", blockerSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Intake/Sim/Blocker/CurrentAmps", blockerSim.getCurrentDrawAmps());
    Logger.recordOutput("Intake/Sim/Blocker/AppliedVoltage", blockerSimState.getMotorVoltage());
  }

  public void updatePivotSimStates() {
    // Find current state of sim in radians from 0 point
    double simPositionRads = pivotSim.getAngularPositionRad();
    Logger.recordOutput("Intake/Sim/SimPivotPositionRadians", simPositionRads);
    // Mutate rotor position
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / IntakeConstants.PIVOT_GEAR_RATIO;
    lastRotations.set(rotorPosition);
    pivotSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Intake/Sim/setPivotRawRotorPosition", rotorPosition);
    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(pivotSim.getAngularVelocityRadPerSec())
            / IntakeConstants.PIVOT_GEAR_RATIO;
    lastRPS.set(rotorVel);
    pivotSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput(
        "Intake/Sim/SimulatorPivotVelocityRadS", pivotSim.getAngularVelocityRadPerSec());
  }

  public void updateRollerSimStates() {
    // Find current state of sim in radians from 0 point
    double simPositionRads = rollerSim.getAngularPositionRad();
    Logger.recordOutput("Intake/Sim/SimRollerPositionRadians", simPositionRads);
    // Mutate rotor position
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / IntakeConstants.ROLLER_GEAR_RATIO;
    rollerSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Intake/Sim/setRollerRawRotorPosition", rotorPosition);
    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec())
            / IntakeConstants.ROLLER_GEAR_RATIO;
    rollerSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Intake/Sim/SimulatorRollerVelocity", rotorVel);
  }

  public void updateL1BlockerSimStates() {
    // Find current state of sim in radians from 0 point
    double simPositionRads = blockerSim.getAngularPositionRad();
    Logger.recordOutput("Intake/Sim/SimBlockerPositionRadians", simPositionRads);
    // Mutate rotor position
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / IntakeConstants.L1_BAR_GEAR_RATIO;
    blockerSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Intake/Sim/setBlockerRawRotorPosition", rotorPosition);
    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(blockerSim.getAngularVelocityRadPerSec())
            / IntakeConstants.L1_BAR_GEAR_RATIO;
    blockerSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Intake/Sim/SimulatorBlockerVelocity", rotorVel);
  }

  // Clean up resources when simulation ends
  public void close() {
    simNotifier.close();
  }
}
