package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class IntakeConstants {

        private IntakeConstants() {} // Prevent instantiation

        // ===== CAN Bus =====
        public static final String CANIVORE_NAME = "Jeffery";
        public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

        // ===== CAN IDs =====
        public static final int INTAKE_MOTOR = 3; //Change when we find the CAN ID for the intake motor
        public static final int PIVOT_MOTOR = 2; //Change when we find the CAN ID for the pivot motor

        // Speed presets for intake motor (rotations per second)
        public static final double INTAKE_VELOCITY = 50.0; //Change when we find the speed we want it at
        public static final double EJECT_VELOCITY = -30.9; //Change when we find the speed we want it at
        public static final double IDLE_VELOCITY = 0.0;

      // PID constants for pivot motor (these will need to be tuned)
        public static final double kP = 30;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double kS = 2.0;
        public static final double kV = 4.5;
        public static final double kA = 0.01;
        public static final double kG = 4.25;

        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        // Motion Magic constants
        public static final int CRUISE_VELOCITY = 15;
        public static final int ACCELERATION = 20;
        public static final int JERK = 20;

        // Pivot preset positions (rotor rotations — tune on robot)
        public static final double STOW_POSITION = 0.0;
        public static final double INTAKE_POSITION = 5.0;
        public static final double TRAVEL_POSITION = 3.5; // slightly above intake, safe for ramp

        // Position tolerance for "at target" checks (rotations)
        public static final double POSITION_TOLERANCE = 0.1;
}
