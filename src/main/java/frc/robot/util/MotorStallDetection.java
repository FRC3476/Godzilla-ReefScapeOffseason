package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

public class MotorStallDetection {
    public boolean isMotorStalled(TalonFX motor, double currentLimitAmps, double velocityLimitRPS) {
        return motor.getStatorCurrent().getValueAsDouble() > currentLimitAmps && motor.getVelocity().getValueAsDouble() < velocityLimitRPS;
    }
}

