package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystems;

public class IntakeCommands extends Command {

    // ====== INTAKE WHEEL COMMANDS ======
    public static Command intake(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.intake();
    }

    public static Command eject(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.eject();
    }

    public static Command idle(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.idle();
    }

    // ====== PIVOT PRESET COMMANDS ======
    public static Command pivotToStow(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotToStow();
    }

    public static Command pivotToIntake(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotToIntake();
    }

    public static Command pivotToTravel(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotToTravel();
    }
}
