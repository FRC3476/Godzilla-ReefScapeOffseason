package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystems.CanCoderInputsAutoLogged;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.canDevice.CanCoderIO;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithCanCoder;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.climb.ClimberOld.ClimbState;
import frc.robot.util.LoggedTunableNumber;

public class ClimberPivot 
    extends ServoMotorSubsystem<
        MotorInputsAutoLogged, MotorIO> {
    private final RobotState robotState;
    private boolean isZeroed = false;
    private static final LoggedTunableNumber  climbTestVolts =
        new LoggedTunableNumber("Climber/ClimberTestVolts", 1.0);
        
    public ClimberPivot(ServoMotorSubsystemConfig config, MotorIO io, RobotState robotState) {
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

    public Command climbDeploy() {
        return climbDeployToPosition(
                ClimbConstants.CLIMB_DEPLOY_POSITION, ClimbConstants.CLIMB_DEPLOY_VOLTAGE)
            .alongWith(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.DEPLOYING)))
            .andThen(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.DEPLOYED)));
    }

    public Command climbClimb() {
        return climbDeployToPosition(
                ClimbConstants.CLIMB_CLIMB_POSITION, ClimbConstants.CLIMB_CLIMB_VOLTAGE)
            .alongWith(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.CLIMBING)))
            .andThen(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.CLIMBED)));
    }

    public Command climbDeployToPosition(double position, double voltage) {
        if (inputs.data.positionRads() > position) {
        return climbSTOP();
        }
        return climbOut(voltage)
            .until(() -> inputs.data.positionRads() > position)
            .andThen(climbSTOP());
    }

    public Command climbDeployToPosition(double position) {
        if (inputs.data.positionRads() > position) {
        return climbSTOP();
        }
        return climbOut().until(() -> inputs.data.positionRads() > position).andThen(climbSTOP());
    }

    public Command climbOut(double voltage) {
        return voltageCommand(() -> voltage);
    }

    public Command climbOut() {
        return voltageCommand(() -> climbTestVolts.getAsDouble());
    }

    public Command climbSTOP() {
        return voltageCommand(() -> 0);
    
    }

    
}