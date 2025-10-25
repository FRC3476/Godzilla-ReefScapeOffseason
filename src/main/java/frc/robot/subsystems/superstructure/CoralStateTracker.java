package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class CoralStateTracker {
  public enum CoralPosition {
    NONE,
    AT_INTAKE,
    GOING_TO_FEEDER,
    AT_BACK_FEEDER,
    AT_FRONT_FEEDER,
    AT_FIRST_END_EFFECTOR,
    AT_SECOND_END_EFFECTOR,
    STAGED_IN_END_EFFECTOR
  }

  private static final SendableChooser<CoralPosition> coralStateOverride = new SendableChooser<>();

  private static final double TIMEOUT_SECONDS = 0.5;

  private static CoralPosition currentPosition = CoralPosition.NONE;
  private static double lastTransitionTime = Timer.getFPGATimestamp();

  private static boolean intakeTriggered = false;
  private static boolean backFeederTriggered = false;
  private static boolean frontFeederTriggered = false;
  private static boolean firstEndEffectorTriggered = false;
  private static boolean secondEndEffectorTriggered = false;

  private static CoralStateTracker instance = new CoralStateTracker();

  private CoralStateTracker() {
    coralStateOverride.setDefaultOption("Default", null);
    coralStateOverride.addOption("None", CoralPosition.NONE);
    coralStateOverride.addOption("At Intake", CoralPosition.AT_INTAKE);
    coralStateOverride.addOption("Going to Feeder", CoralPosition.GOING_TO_FEEDER);
    coralStateOverride.addOption("At BACK Feeder", CoralPosition.AT_BACK_FEEDER);
    coralStateOverride.addOption("At FRONT Feeder", CoralPosition.AT_FRONT_FEEDER);
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

  public static void updateBackFeeder(boolean value) {
    backFeederTriggered = value;
    recalcState();
  }

  public static void updateFrontFeeder(boolean value) {
    frontFeederTriggered = value;
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
        if (frontFeederTriggered) {
          currentPosition = CoralPosition.AT_FRONT_FEEDER;
          lastTransitionTime = now;
        }
        if (backFeederTriggered) {
          currentPosition = CoralPosition.AT_BACK_FEEDER;
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
        if (frontFeederTriggered) {
          currentPosition = CoralPosition.AT_FRONT_FEEDER;
          lastTransitionTime = now;
        }
        if (backFeederTriggered) {
          currentPosition = CoralPosition.AT_BACK_FEEDER;
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
        } else if (frontFeederTriggered) {
          currentPosition = CoralPosition.AT_FRONT_FEEDER;
          lastTransitionTime = now;
        } else if (backFeederTriggered) {
          currentPosition = CoralPosition.AT_BACK_FEEDER;
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

      case AT_FRONT_FEEDER:
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (backFeederTriggered) {
          currentPosition = CoralPosition.AT_BACK_FEEDER;
          lastTransitionTime = now;
        } else if (frontFeederTriggered) {
          lastTransitionTime = now;
        } else if (intakeTriggered) {
          currentPosition = CoralPosition.AT_INTAKE;
          lastTransitionTime = now;
        } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
          currentPosition = CoralPosition.NONE;
        }
        break;

      case AT_BACK_FEEDER:
        if (firstEndEffectorTriggered && secondEndEffectorTriggered) {
          currentPosition = CoralPosition.STAGED_IN_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (secondEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_SECOND_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (firstEndEffectorTriggered) {
          currentPosition = CoralPosition.AT_FIRST_END_EFFECTOR;
          lastTransitionTime = now;
        } else if (backFeederTriggered) {
          lastTransitionTime = now;
        } else if (frontFeederTriggered) {
          currentPosition = CoralPosition.AT_FRONT_FEEDER;
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

  /**
   * Returns true if coral is in the end effector: AT_FIRST_END_EFFECTOR, AT_SECOND_END_EFFECTOR,
   * STAGED_IN_END_EFFECTOR
   */
  public static boolean IsCoralInEndEffector() {
    if (CoralStateTracker.getCurrentPosition() == CoralPosition.AT_FIRST_END_EFFECTOR
        || CoralStateTracker.getCurrentPosition() == CoralPosition.AT_SECOND_END_EFFECTOR
        || CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR) {
      return true;
    } else {
      return false;
    }
  }

  public static void forceSet(CoralPosition newState) {
    currentPosition = newState;
    lastTransitionTime = Timer.getFPGATimestamp();
  }

  public static Trigger isAtFrontFeederTrigger() {
    return new Trigger(() -> getCurrentPosition() == CoralPosition.AT_FRONT_FEEDER);
  }

  public static Trigger isAtIntakeTrigger() {
    return new Trigger(() -> getCurrentPosition() == CoralPosition.AT_INTAKE);
  }

  public static Trigger isStuckAtFrontFeederTrigger() {
    return isAtFrontFeederTrigger().debounce(0.5);
  }

  public static Trigger isStuckAtIntakeTrigger() {
    return isAtIntakeTrigger().debounce(0.5);
  }

  public static boolean hasCoral() {
    switch (currentPosition) {
      case AT_INTAKE,
          GOING_TO_FEEDER,
          AT_FRONT_FEEDER,
          AT_BACK_FEEDER,
          AT_FIRST_END_EFFECTOR,
          AT_SECOND_END_EFFECTOR,
          STAGED_IN_END_EFFECTOR:
        return true;
      default:
        return false;
    }
  }
}
