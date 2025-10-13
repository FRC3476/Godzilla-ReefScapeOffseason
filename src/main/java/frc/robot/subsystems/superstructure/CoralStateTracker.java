package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private static final SendableChooser<CoralPosition> coralStateOverride = new SendableChooser<>();

  private static final double TIMEOUT_SECONDS = 0.5;

  private static CoralPosition currentPosition = CoralPosition.NONE;
  private static double lastTransitionTime = Timer.getFPGATimestamp();

  private static boolean intakeTriggered = false;
  private static boolean feederTriggered = false;
  private static boolean firstEndEffectorTriggered = false;
  private static boolean secondEndEffectorTriggered = false;

  private static CoralStateTracker instance = new CoralStateTracker();

  private CoralStateTracker() {
    coralStateOverride.setDefaultOption("Default", null);
    coralStateOverride.addOption("None", CoralPosition.NONE);
    coralStateOverride.addOption("At Intake", CoralPosition.AT_INTAKE);
    coralStateOverride.addOption("Going to Feeder", CoralPosition.GOING_TO_FEEDER);
    coralStateOverride.addOption("At Feeder", CoralPosition.AT_FEEDER);
    coralStateOverride.addOption("At First End Effector", CoralPosition.AT_FIRST_END_EFFECTOR);
    coralStateOverride.addOption("At Second End Effector", CoralPosition.AT_SECOND_END_EFFECTOR);
    coralStateOverride.addOption("Staged in End Effector", CoralPosition.STAGED_IN_END_EFFECTOR);
    SmartDashboard.putData("State Overrides/CoralState Override", coralStateOverride);
  }

  public static CoralStateTracker getInstance() {
    return instance;
  }

  public static void updateIntake(boolean value) {
    intakeTriggered = value;
    recalcState();
  }

  public static void updateFeeder(boolean value) {
    feederTriggered = value;
    recalcState();
  }

  public static void updateFirstEndEffector(boolean value) {
    firstEndEffectorTriggered = value;
    recalcState();
  }

  public static void updateSecondEndEffector(boolean value) {
    secondEndEffectorTriggered = value;
    recalcState();
  }

  public static void updateBothEndEffectors(boolean firstValue, boolean secondValue) {
    firstEndEffectorTriggered = firstValue;
    secondEndEffectorTriggered = secondValue;
    recalcState();
  }

  private static void recalcState() {
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
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
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
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
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
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
          lastTransitionTime = now;
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
    if (coralStateOverride.getSelected() != null) {
      return coralStateOverride.getSelected();
    }
    return currentPosition;
  }

  public static void forceSet(CoralPosition newState) {
    currentPosition = newState;
    lastTransitionTime = Timer.getFPGATimestamp();
  }
}
