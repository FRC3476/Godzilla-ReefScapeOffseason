// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

  }

  //====================Intake (3_)====================
  public static class IntakeConstants {

    public static final int intakePivotPort = 30;
    public static final int intakeRollerPort = 31;
    public static final int intakeL1StopperPort = 32;

  }

  //====================Elevator (4_)====================
  public static class ElevatorConstants {

    public static final int elevatorRightPort = 40;
    public static final int elevatorLeftPort = 41;
    public static final int elevatorExtraPort = 42;

      
  }

  //====================End Effector (5_)====================
  public static class EndEffectorConstants {
    public static final int endEffectorPivotPort = 50;
    public static final int endEffectorRollerPort = 51; 
  }

  //====================Climb (6_)====================
  public static class ClimbConstants {
    public static final int climbPort = 60;
    
  }

  //====================Indexer (7_)====================
  public static class IndexerConstants {
    public static final int indexerRightPort = 70;
    public static final int indexerLeftPort = 71;
    public static final int indexerCANrange = 72;

  }
  
  



  public record PIDgains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {

        //Pure PID (the average use case)
        public PIDgains(double kP, double kI, double kD) {
            this(kP, kI, kD, 0, 0, 0,0);
        }

        //Arm/Elevator PID (we don't care about voltage or acceleration)
        public PIDgains(double kP, double kI, double kD, double kS, double kG){
            this(kP, kI, kD, kS, 0, 0, kG);
        }

        //Flywheels/Wheels (since kG is not an issue)
        public PIDgains(double kP, double kI, double kD, double kS, double kV, double kA){
            this(kP, kI, kD, kS, kV, kA, 0);
        }
    }
}
