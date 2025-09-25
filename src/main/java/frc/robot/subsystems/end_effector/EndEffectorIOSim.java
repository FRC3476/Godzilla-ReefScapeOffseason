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
  protected DCMotorSim rollerSim;
  protected Notifier simNotifier;

  private final TalonFXSimState pivotSimState;
  private final TalonFXSimState rollerSimState;

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
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.01, 1.0 / EndEffectorConstants.CORAL_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    pivotTalonFX.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    rollerTalonFX.getSimState().Orientation = ChassisReference.Clockwise_Positive;

    pivotSimState = pivotTalonFX.getSimState();
    rollerSimState = rollerTalonFX.getSimState();

    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  private void updateSimState() {
    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    double pivotVoltage = pivotSimState.getMotorVoltage();
    double rollerVoltage = rollerSimState.getMotorVoltage();

    pivotSim.setInputVoltage(pivotVoltage);
    rollerSim.setInputVoltage(rollerVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.getCurrentDrawAmps()));

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    pivotSim.update(dt);
    rollerSim.update(dt);

    updatePivotSimStates();
    updateRollerSimStates();

    firstCoralCANRange.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    secondCoralCANRange.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    firstCoralCANRange.getSimState().setDistance(0.1);
    secondCoralCANRange.getSimState().setDistance(0.1);

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

  private void updateRollerSimStates() {
    double simPositionRads = rollerSim.getAngularPositionRad();
    Logger.recordOutput("EndEffector/Sim/SimRollerPositionRadians", simPositionRads);
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / EndEffectorConstants.CORAL_GEAR_RATIO;
    rollerSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("EndEffector/Sim/setRollerRawRotorPosition", rotorPosition);
    double rotorVel =
        Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec())
            / EndEffectorConstants.CORAL_GEAR_RATIO;
    rollerSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput(
        "EndEffector/Sim/SimRollerVelocityRadS", rollerSim.getAngularVelocityRadPerSec());
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
  }

  public void close() {
    simNotifier.close();
  }
}
