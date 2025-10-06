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
import org.littletonrobotics.junction.Logger;

public class ClawIOSim extends ClawIOReal {
  protected DCMotorSim rollerSim;

  private final TalonFXSimState rollerSimState;

  protected double lastUpdateTimestamp = 0.0;

  protected Notifier simNotifier;

  public ClawIOSim() {

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                EndEffectorConstants.ROLLER_MOI,
                1.0 / EndEffectorConstants.ALGAE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    rollerTalonFX.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

    rollerSimState = rollerTalonFX.getSimState();

    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  private void updateSimState() {
    rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double rollerVoltage = rollerSimState.getMotorVoltage();
    rollerSim.setInputVoltage(rollerVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.getCurrentDrawAmps()));

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    rollerSim.update(dt);

    updateRollerSimStates();

    logSimulationData();
  }

  private void updateRollerSimStates() {
    // Find current state of sim in radians from 0 point
    double simPositionRads = rollerSim.getAngularPositionRad();
    Logger.recordOutput("Claw/Sim/SimRollerPositionRadians", simPositionRads);
    // Mutate rotor position
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / EndEffectorConstants.ALGAE_GEAR_RATIO;
    rollerSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Claw/Sim/setRollerRawRotorPosition", rotorPosition);
    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec())
            / EndEffectorConstants.ALGAE_GEAR_RATIO;
    rollerSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Claw/Sim/SimulatorRollerVelocity", rotorVel);
  }

  private void logSimulationData() {
    Logger.recordOutput("Claw/Sim/Roller/PositionRad", rollerSim.getAngularPositionRad());
    Logger.recordOutput("Claw/Sim/Roller/VelocityRPS", rollerSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Claw/Sim/Roller/CurrentAmps", rollerSim.getCurrentDrawAmps());
    Logger.recordOutput("Claw/Sim/Roller/AppliedVoltage", rollerSimState.getMotorVoltage());
  }

  public void close() {
    simNotifier.close();
  }
}
