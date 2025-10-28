// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.superstructure.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score extends SequentialCommandGroup {
  /** Creates a new score. */
  public Score(
      Superstructure superstructure, Claw claw, RobotState robotState, RobotContainer container) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        new WaitUntilCommand(() -> container.getDrive().isRobotStable()),
        new ConditionalCommand(
            claw.setClawStateCommand(ClawState.SCORING_L1).asProxy(),
            claw.setClawStateCommand(ClawState.SCORING).asProxy(),
            () -> robotState.isL1Mode()),
        new WaitUntilCommand(() -> CoralStateTracker.getCurrentPosition() == CoralPosition.NONE)
            .withTimeout(3),
        superstructure.setStateCommand(() -> robotState.getFadeawayState(), "Aim fade").asProxy(),
        claw.setClawStateCommand(ClawState.IDLE).asProxy());
  }
}
