package frc.robot.subsystems.feeder;

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
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class FeederIOSim extends FeederIOReal {
  // Note: This is currently for the right roller only, since the left roller is a follower
  protected DCMotorSim rollerSim;

  // TalonFX simulation states
  private final TalonFXSimState rollerSimState;

  // Simulation state tracking
  protected double lastUpdateTimestamp = 0.0;

  protected Notifier simNotifier;

  // =============================

  public FeederIOSim() {

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                FeederConstants.ROLLER_MOI,
                1.0 / FeederConstants.ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60(2));

    // sim states
    rightRoller.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

    rollerSimState = rightRoller.getSimState();

    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005); // 5ms update rate
  }

  private void updateSimState() {
    // set voltage
    rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // get voltage
    double rollerVoltage = rollerSimState.getMotorVoltage();
    // add voltages to sim
    rollerSim.setInputVoltage(rollerVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.getCurrentDrawAmps()));

    // Update simulation models
    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    rollerSim.update(dt);

    updateRollerSimStates();

    // Log simulation data
    logSimulationData();
  }

  private void updateRollerSimStates() {
    // Find current state of sim in radians from 0 point
    double simPositionRads = rollerSim.getAngularPositionRad();
    Logger.recordOutput("Feeder/Sim/SimRollerPositionRadians", simPositionRads);
    // Mutate rotor position
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / IntakeConstants.ROLLER_GEAR_RATIO;
    rollerSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Feeder/Sim/setRollerRawRotorPosition", rotorPosition);
    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec())
            / IntakeConstants.ROLLER_GEAR_RATIO;
    rollerSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Feeder/Sim/SimulatorRollerVelocity", rotorVel);
  }

  private void logSimulationData() {
    // Log roller simulation data
    Logger.recordOutput("Feeder/Sim/Roller/PositionRad", rollerSim.getAngularPositionRad());
    Logger.recordOutput("Feeder/Sim/Roller/VelocityRPS", rollerSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Feeder/Sim/Roller/CurrentAmps", rollerSim.getCurrentDrawAmps());
    Logger.recordOutput("Feeder/Sim/Roller/AppliedVoltage", rollerSimState.getMotorVoltage());
  }

  public void close() {
    simNotifier.close();
  }
}
