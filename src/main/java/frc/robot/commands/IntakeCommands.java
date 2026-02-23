package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystems;

public final class IntakeCommands {

    // ====== INTAKE WHEEL COMMANDS ======
    public static Command intake(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.intake().withName("IntakeCommand");
    }

    public static Command eject(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.eject().withName("EjectCommand");
    }

    public static Command idle(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.idle().withName("IdleCommand");
    }

    // ====== PIVOT PRESET COMMANDS ======
    public static Command pivotToStow(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotToStow().withName("PivotToStowCommand");
    }

    public static Command pivotToIntake(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotToIntake().withName("PivotToIntakeCommand");
    }

    public static Command pivotToTravel(IntakeSubsystems intakeSubsystems) {
        return intakeSubsystems.pivotToTravel().withName("PivotToTravelCommand");
    }

    // =====Advance Commands=====
    public static Command startingIntakeSequence(IntakeSubsystems intakeSubsystems) {
        return Commands.sequence(
            intakeSubsystems.pivotToIntake(),
            intakeSubsystems.waitUntilPivotAtTarget(),
            intakeSubsystems.intake()
        ).withName("IntakeSequenceCommand");
    }

    public static Command endingIntakeSequence(IntakeSubsystems intakeSubsystems) {
        return Commands.sequence(
            intakeSubsystems.idle(),
            intakeSubsystems.pivotToTravel()
        ).withName("EndIntakeSequenceCommand");
    }
}
