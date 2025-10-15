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

public class ClawIOSim extends ClawIOReal {
  protected DCMotorSim climbRollerSim;

  private final TalonFXSimState climbRollerSimState;

  protected double lastUpdateTimestamp = 0.0;

  protected Notifier simNotifier;

  public ClawIOSim() {

    climbRollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                EndEffectorConstants.ROLLER_MOI,
                1.0 / EndEffectorConstants.ALGAE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    rollerTalonFX.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

    climbRollerSimState = rollerTalonFX.getSimState();

    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  private void updateSimState() {
    climbRollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double rollerVoltage = climbRollerSimState.getMotorVoltage();
    climbRollerSim.setInputVoltage(rollerVoltage);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(climbRollerSim.getCurrentDrawAmps()));

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    climbRollerSim.update(dt);

    updateRollerSimStates();

    logSimulationData();
  }

  private void updateRollerSimStates() {
    // Find current state of sim in radians from 0 point
    double simPositionRads = climbRollerSim.getAngularPositionRad();
    Logger.recordOutput("ClimbRoller/Sim/SimRollerPositionRadians", simPositionRads);
    // Mutate rotor position
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) / EndEffectorConstants.ALGAE_GEAR_RATIO; //TODO change
    climbRollerSimState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("ClimbRoller/Sim/setRollerRawRotorPosition", rotorPosition);
    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(climbRollerSim.getAngularVelocityRadPerSec())
            / EndEffectorConstants.ALGAE_GEAR_RATIO;
    climbRollerSimState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Claw/Sim/SimulatorRollerVelocity", rotorVel);
  }

  private void logSimulationData() {
    Logger.recordOutput("CLimbRoller/Sim/PositionRad", climbRollerSim.getAngularPositionRad());
    Logger.recordOutput("CLimbRoller/Sim/VelocityRPS", climbRollerSim.getAngularVelocityRadPerSec());
    Logger.recordOutput("CLimbRoller/Sim/CurrentAmps", climbRollerSim.getCurrentDrawAmps());
    Logger.recordOutput("CLimbRoller/Sim/AppliedVoltage", climbRollerSimState.getMotorVoltage());
  }

  public void close() {
    simNotifier.close();
  }
}
