package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.sim.CANcoderSimState;
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
import frc.robot.Constants.EndEffectorConstants;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSim extends EndEffectorIOReal {

  protected DCMotorSim pivotSim;
  protected Notifier simNotifier;

  private final TalonFXSimState pivotSimState;
  private final CANcoderSimState cancoderSimState;

  protected double lastUpdateTimestamp = 0.0;

  protected AtomicReference<Double> pivotLastRotations = new AtomicReference<>((double) 0.0);
  protected AtomicReference<Double> pivotLastRPS = new AtomicReference<>((double) 0.0);

  public EndEffectorIOSim() {
    super();

    // DCMotorSim gearing parameter is mechanism-to-motor, so we use PIVOT_GEAR_RATIO directly
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.01, EndEffectorConstants.PIVOT_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    pivotTalonFX.getSimState().Orientation = ChassisReference.Clockwise_Positive;

    pivotSimState = pivotTalonFX.getSimState();

    // Get CANcoder simulation state to simulate the remote sensor
    cancoderSimState = pivotCancoder.getSimState();
    cancoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    // Initialize CANcoder absolute position to match starting mechanism position
    // This is critical for matching real robot behavior where absolute position is used
    double initialMechanismRot = 0.0; // Assuming mechanism starts at 0
    double initialAbsolutePosition = initialMechanismRot * EndEffectorConstants.PIVOT_STM;
    cancoderSimState.setRawPosition(initialAbsolutePosition);
    cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Initialize timestamp before starting simulation updates
    lastUpdateTimestamp = Timer.getFPGATimestamp();

    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  private void updateSimState() {
    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    double pivotVoltage = pivotSimState.getMotorVoltage();

    // Log control mode and setpoint for debugging
    Logger.recordOutput(
        "EndEffector/Sim/ControlMode", pivotTalonFX.getControlMode().getValue().toString());
    Logger.recordOutput(
        "EndEffector/Sim/ClosedLoopReference",
        pivotTalonFX.getClosedLoopReference().getValueAsDouble());

    pivotSim.setInputVoltage(pivotVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    pivotSim.update(dt);

    updatePivotSimStates();

    logSimulationData();
  }

  private void updatePivotSimStates() {
    // Get mechanism position from simulation (in radians)
    double mechanismPositionRad = pivotSim.getAngularPositionRad();
    double mechanismVelocityRadPerSec = pivotSim.getAngularVelocityRadPerSec();

    // Convert mechanism position to rotations
    double mechanismPositionRot = Units.radiansToRotations(mechanismPositionRad);
    double mechanismVelocityRPS = Units.radiansToRotations(mechanismVelocityRadPerSec);

    // Calculate rotor position: mechanism * (RTS * STM)
    // Rotor spins faster than mechanism by the total gear ratio
    double rotorPosition = mechanismPositionRot / EndEffectorConstants.PIVOT_GEAR_RATIO;
    double rotorVelocity = mechanismVelocityRPS / EndEffectorConstants.PIVOT_GEAR_RATIO;

    // Store in AtomicReference for thread-safe access
    pivotLastRotations.set(rotorPosition);
    pivotLastRPS.set(rotorVelocity);

    // Update TalonFX simulation state
    pivotSimState.setRawRotorPosition(rotorPosition);
    pivotSimState.setRotorVelocity(rotorVelocity);

    // Update CANcoder simulation: CANcoder measures mechanism position
    // CANcoder position = mechanism * STM (sensor-to-mechanism ratio)
    double cancoderPosition = mechanismPositionRot * EndEffectorConstants.PIVOT_STM;
    double cancoderVelocity = mechanismVelocityRPS * EndEffectorConstants.PIVOT_STM;
    cancoderSimState.setRawPosition(cancoderPosition);
    cancoderSimState.setVelocity(cancoderVelocity);

    // Logging
    Logger.recordOutput("EndEffector/Sim/MechanismPositionRot", mechanismPositionRot);
    Logger.recordOutput("EndEffector/Sim/RotorPosition", rotorPosition);
    Logger.recordOutput("EndEffector/Sim/CANcoderPosition", cancoderPosition);
    Logger.recordOutput("EndEffector/Sim/SimPivotVelocityRadS", mechanismVelocityRadPerSec);
  }

  private void logSimulationData() {
    // Log pivot simulation data
    Logger.recordOutput(
        "Endeffector/Sim/Pivot/PositionRot",
        Units.radiansToRotations(pivotSim.getAngularPositionRad()));
    Logger.recordOutput(
        "Endeffector/Sim/Pivot/VelocityRPS", pivotSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Endeffector/Sim/Pivot/CurrentAmps", pivotSim.getCurrentDrawAmps());
    Logger.recordOutput("Endeffector/Sim/Pivot/AppliedVoltage", pivotSimState.getMotorVoltage());
  }

  public void close() {
    simNotifier.close();
  }
}
