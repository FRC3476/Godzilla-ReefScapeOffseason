package frc.robot.subsystems.end_effector;

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

  protected double lastUpdateTimestamp = 0.0;

  protected AtomicReference<Double> pivotLastRotations = new AtomicReference<>((double) 0.0);
  protected AtomicReference<Double> pivotLastRPS = new AtomicReference<>((double) 0.0);

  public EndEffectorIOSim() {
    super();

    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.01, 1.0 / EndEffectorConstants.PIVOT_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    pivotTalonFX.getSimState().Orientation = ChassisReference.Clockwise_Positive;

    pivotSimState = pivotTalonFX.getSimState();

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
    double simPositionRads = pivotSim.getAngularPositionRad();
    Logger.recordOutput("EndEffector/Sim/SimPivotPositionRadians", simPositionRads);
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / EndEffectorConstants.PIVOT_GEAR_RATIO;
    pivotLastRotations.set(rotorPosition);
    pivotSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("EndEffector/Sim/setPivotRawRotorPosition", rotorPosition);
    double rotorVel =
        Units.radiansToRotations(pivotSim.getAngularVelocityRadPerSec())
            / EndEffectorConstants.PIVOT_GEAR_RATIO;
    pivotLastRPS.set(rotorVel);
    pivotSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput(
        "EndEffector/Sim/SimPivotVelocityRadS", pivotSim.getAngularVelocityRadPerSec());
  }

  private void logSimulationData() {
    // Log pivot simulation data
    Logger.recordOutput("Intake/Sim/Pivot/PositionRad", pivotSim.getAngularPositionRad());
    Logger.recordOutput("Intake/Sim/Pivot/VelocityRPS", pivotSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Intake/Sim/Pivot/CurrentAmps", pivotSim.getCurrentDrawAmps());
    Logger.recordOutput("Intake/Sim/Pivot/AppliedVoltage", pivotSimState.getMotorVoltage());
  }

  public void close() {
    simNotifier.close();
  }
}
