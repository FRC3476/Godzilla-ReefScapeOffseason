package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.LoggedTunableNumber;

public class ClimbRoller
    extends ServoMotorSubsystem<
        MotorInputsAutoLogged, MotorIO> {
    private final RobotState robotState;
    private boolean isZeroed = false;
    private static final LoggedTunableNumber rollerTestVolts =
      new LoggedTunableNumber(
          "ClimbRoller/TunableVolts",
          1.0);
    private static final LoggedTunableNumber rollerTestHoldingCageAmps =
      new LoggedTunableNumber("ClimbRoller/HoldingCageAmps", ClimbConstants.ROLLER_HOLD_CAGE_AMPS);
    
    
    private boolean climbing = false;

    public static LimitSwitchState limitSwitchState = LimitSwitchState.NONE;

    private static double latchedTimestamp = 0.0;

    public ClimbRoller(ServoMotorSubsystemConfig config, MotorIO io, RobotState robotState) {
    super(config, new MotorInputsAutoLogged(), io);
    this.robotState = robotState;
    setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName(getName() + " Default Command Hold Position")
            .ignoringDisable(true));
    }

    public enum ClimbState {
        STOWED,
        DEPLOYING,
        DEPLOYED,
        CLIMBING,
        CLIMBED
    }

    
    public enum LimitSwitchState {
        NONE,
        LATCHING,
        LATCHED
    }
    
    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput(getName() + "/positionRotations", getCurrentPosition());
    }

    public Command setRollerVoltage(double voltage) {
        return setVoltageOutput(voltage)
    }

    public Command rollerFWD() {
        return voltageCommand(() -> rollerTestVolts.getAsDouble());
    }

    public Command rollerRVS() {
        return voltageCommand(() -> -rollerTestVolts.getAsDouble());
    }

    public Command rollerSTOP() {
        return voltageCommand(() -> 0);
    }

    public Command holdCage() {
        return setTorqueCurrentFOC(() -> rollerTestHoldingCageAmps.get());
    }

    public boolean getClimbing() {
        return climbing;
    }
    public boolean hasCage() {
        return ClimbRoller.limitSwitchState == ClimbRoller.limitSwitchState.LATCHED
            && climbing
            && io.checkRollerStalled();
    }

}
