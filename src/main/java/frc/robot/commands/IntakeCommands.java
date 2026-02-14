package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystems;

@SuppressWarnings("unused")

public class IntakeCommands extends Command {

    // ====== BASIC COMMANDS ======
    public static Command intake(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.intake();
    }

    public static Command eject(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.eject();
    }

    public static Command idle(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.idle();
    }

    public static Command pivotUp(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotUp();
    }

    public static Command pivotDown(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotDown();
    }

    
}
