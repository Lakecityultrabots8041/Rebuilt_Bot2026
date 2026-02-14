package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
    
        private IntakeConstants() {} // Prevent instantiation
    
        // ===== CAN Bus =====
        public static final String CANIVORE_NAME = "Jeffery";
        public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);
    
        // ===== CAN IDs =====
        public static final int INTAKE_MOTOR = 4; //Change when we find the CAN ID for the intake motor
        public static final int PIVOT_MOTOR = 5; //Change when we find the CAN ID for the pivot motor

        // Speed presets for intake motor (rotations per second)
        public static final double INTAKE_VELOCITY = 50.0; //Change when we find the speed we want it at
        public static final double EJECT_VELOCITY = -30.9; //Change when we find the speed we want it at
        public static final double IDLE_VELOCITY = 0.0;

        // Speed presets for pivot motor (rotations per second)
        public static final double PIVOT_UP_VELOCITY = 20.0; //Change when we find the speed we want it at
        public static final double PIVOT_DOWN_VELOCITY = -20.0; //Change when we find the speed we want it at
        public static final double PIVOT_IDLE_VELOCITY = 0.0;

}
