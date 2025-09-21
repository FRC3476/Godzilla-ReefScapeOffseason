package frc.robot.test;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;

class IntakeMotorTest {
    private Intake intake;

    @BeforeEach
    void setUp() {
        intake = new Intake(new IntakeIOSim());
        assertNotNull(intake);
    }

    @Test
    void testPivot() {
        var cmd = intake.movePivotDown();
        assertNotNull(cmd);
        cmd.schedule();
        intake.setIntakeState(IntakeConstants.IntakeState.STOW).schedule();
        intake.setIntakeState(IntakeConstants.IntakeState.SCORING).schedule();
    }

    @Test
    void testRoller() {
        var cmd = intake.intakeFWD();
        assertNotNull(cmd);
        cmd.schedule();
        intake.intakeSTOP().schedule();
        intake.intakeRVS().schedule();
        intake.intakeSTOP().schedule();
    }

    @Test
    void testBlocker() {
        var cmd = intake.engageCoralL1();
        assertNotNull(cmd);
        cmd.schedule();
        intake.disengageCoralL1().schedule();
        intake.engageCoralL1().schedule();
    }

    @Test
    void testStates() {
        var cmd = intake.setIntakeState(IntakeConstants.IntakeState.INTAKE);
        assertNotNull(cmd);
        cmd.schedule();
        intake.setIntakeState(IntakeConstants.IntakeState.SCORING).schedule();
        intake.setIntakeState(IntakeConstants.IntakeState.IDLE).schedule();
    }
}