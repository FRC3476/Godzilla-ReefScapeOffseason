package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class CoralStateTracker {
  public enum CoralPosition {
    NONE,
    AT_INTAKE,
    GOING_TO_FEEDER,
    AT_FEEDER,
    AT_FIRST_END_EFFECTOR,
    AT_SECOND_END_EFFECTOR,
    STAGED_IN_END_EFFECTOR
  }

  private static final double TIMEOUT_SECONDS = 0.5;

  private static CoralPosition currentPosition = CoralPosition.NONE;
  private double lastTransitionTime = Timer.getFPGATimestamp();

  private boolean intakeTriggered = false;
  private boolean feederTriggered = false;
  private boolean firstEndEffectorTriggered = false;
  private boolean secondEndEffectorTriggered = false;

  public void updateIntake(boolean value) {
    intakeTriggered = value;
    recalcState();
  }

  public void updateFeeder(boolean value) {
    feederTriggered = value;
    recalcState();
  }

  public void updateFirstEndEffector(boolean value) {
    firstEndEffectorTriggered = value;
    recalcState();
  }

  public void updateSecondEndEffector(boolean value) {
    secondEndEffectorTriggered = value;
    recalcState();
  }

  private void recalcState() {
    double now = Timer.getFPGATimestamp();

    Logger.recordOutput("CoralStateTracker/lastTransitionTime", lastTransitionTime);

    switch (currentPosition) {
      case NONE:
        if (intakeTriggered) {
          currentPosition = CoralPosition.AT_INTAKE;
          lastTransitionTime = now;
        }
        if (feederTriggered) {
          currentPosition = CoralPosition.AT_FEEDER;
          lastTransitionTime = now;
        }
        break;

      case AT_INTAKE:
        if (intakeTriggered) {
          lastTransitionTime = now;
        } else {
          currentPosition = CoralPosition.GOING_TO_FEEDER;
          lastTransitionTime = now;
        }
        if (feederTriggered) {
          currentPosition = CoralPosition.AT_FEEDER;
          lastTransitionTime = now;
        }
        break;

      case GOING_TO_FEEDER:
        if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (feederTriggered) {
          currentPosition = CoralPosition.AT_FEEDER;
          lastTransitionTime = now;
        } else if (intakeTriggered) {
          currentPosition = CoralPosition.AT_INTAKE;
          lastTransitionTime = now;

        } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
          currentPosition = CoralPosition.NONE;
        }
        break;

      case AT_FEEDER:
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (feederTriggered) {
          lastTransitionTime = now;
        } else if (intakeTriggered) {
          currentPosition = CoralPosition.AT_INTAKE;
          lastTransitionTime = now;
        } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
          currentPosition = CoralPosition.NONE;
        }
        break;

      case AT_FIRST_END_EFFECTOR:
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
          currentPosition = CoralPosition.NONE;
        }
        break;

      case AT_SECOND_END_EFFECTOR:
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          lastTransitionTime = now;
        } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
          currentPosition = CoralPosition.NONE;
        }
        break;

      case STAGED_IN_END_EFFECTOR:
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
          currentPosition = CoralPosition.NONE;
        }
        break;
    }
  }

  public static CoralPosition getCurrentPosition() {
    return currentPosition;
  }

  public void forceSet(CoralPosition newState) {
    currentPosition = newState;
    lastTransitionTime = Timer.getFPGATimestamp();
  }
}
