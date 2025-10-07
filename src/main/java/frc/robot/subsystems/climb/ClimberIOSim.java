package frc.robot.subsystems.climb;

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
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSim extends ClimberIOReal {

  protected DCMotorSim climbSim;

  // TalonFX simulation states
  private final TalonFXSimState climbSimState;

  // Simulation state tracking
  protected double lastUpdateTimestamp = 0.0;

  protected Notifier simNotifier;

  // =============================

  public ClimberIOSim() {

    climbSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), ClimbConstants.climbMOI, 1.0 / ClimbConstants.reduction),
            DCMotor.getKrakenX60(1));

    // sim states
    talon.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

    climbSimState = talon.getSimState();

    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005); // 5ms update rate
  }

  private void updateSimState() {
    // set voltage
    climbSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // get voltage
    double climbVoltage = climbSimState.getMotorVoltage();
    // add voltages to sim
    climbSim.setInputVoltage(climbVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(climbSim.getCurrentDrawAmps()));

    // Update simulation models
    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    climbSim.update(dt);

    updateclimbSimStates();

    // Log simulation data
    logSimulationData();
  }

  private void updateclimbSimStates() {
    // Find current state of sim in radians from 0 point
    double simPositionRads = climbSim.getAngularPositionRad();
    Logger.recordOutput("Climber/Sim/SimclimbPositionRadians", simPositionRads);
    // Mutate rotor position
    double rotorPosition = Units.radiansToRotations(simPositionRads) / ClimbConstants.reduction;
    climbSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Climber/Sim/setclimbRawRotorPosition", rotorPosition);
    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(climbSim.getAngularVelocityRadPerSec()) / ClimbConstants.reduction;
    climbSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Climber/Sim/SimulatorclimbVelocity", rotorVel);
  }

  private void logSimulationData() {
    // Log climb simulation data
    Logger.recordOutput("Climber/Sim/climb/PositionRad", climbSim.getAngularPositionRad());
    Logger.recordOutput("Climber/Sim/climb/VelocityRPS", climbSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("Climber/Sim/climb/CurrentAmps", climbSim.getCurrentDrawAmps());
    Logger.recordOutput("Climber/Sim/climb/AppliedVoltage", climbSimState.getMotorVoltage());
  }

  public void close() {
    simNotifier.close();
  }
}
