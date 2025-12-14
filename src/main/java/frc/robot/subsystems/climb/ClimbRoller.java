package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        
    public ClimbRoller(ServoMotorSubsystemConfig config, MotorIO io, RobotState robotState) {
    super(config, new MotorInputsAutoLogged(), io);
    this.robotState = robotState;
    setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName(getName() + " Default Command Hold Position")
            .ignoringDisable(true));
    }
    
    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput(getName() + "/positionRotations", getCurrentPosition());
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
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
