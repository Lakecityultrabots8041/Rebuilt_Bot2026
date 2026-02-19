package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystems;

public class IntakeCommands extends Command {

    // ====== INTAKE WHEEL COMMANDS ======
    public static Command intake(IntakeSubsystems intakeSubsystems) {
       return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Starting intake command")),
        intakeSubsystems.intake()
       ).withName("IntakeCommand");
        
    }

    public static Command eject(IntakeSubsystems intakeSubsystems) {
        return Commands.runOnce(() -> System.out.println("Starting eject command"))
                .andThen(intakeSubsystems.eject())
                .withName("EjectCommand");
    }

    public static Command idle(IntakeSubsystems intakeSubsystems) {
        return Commands.runOnce(() -> System.out.println("Starting idle command"))
                .andThen(intakeSubsystems.idle())
                .withName("IdleCommand");
    }

    // ====== PIVOT PRESET COMMANDS ======
    public static Command pivotToStow(IntakeSubsystems intakeSubsystems) {
        return Commands.runOnce(() -> System.out.println("Lifting to starting position"))
                .andThen(intakeSubsystems.pivotToStow())
                .withName("PivotToStowCommand");
    }

    public static Command pivotToIntake(IntakeSubsystems intakeSubsystems) {
        return Commands.runOnce(() -> System.out.println("Lowering to intake position"))
                .andThen(intakeSubsystems.pivotToIntake())
                .withName("PivotToIntakeCommand");
    }

    public static Command pivotToTravel(IntakeSubsystems intakeSubsystems) {
        return Commands.runOnce(() -> System.out.println("Lifting to travel position"))
                .andThen(intakeSubsystems.pivotToTravel())
                .withName("PivotToTravelCommand");
    }

    // =====Advance Commands=====
    public static Command startingIntakeSequence(IntakeSubsystems intakeSubsystems) {
        return Commands.sequence(
            Commands.runOnce(() -> System.out.println("Starting Intake Sequence")),
            Commands.waitSeconds(0.5), // Change when we find the exact time it takes to do that action
            Commands.runOnce(() -> System.out.println("Lowering Intake To Intake Position")),
            intakeSubsystems.pivotToIntake(),
            Commands.waitSeconds(0.5), // Change when we find the exact time it takes to do that action
            Commands.runOnce(() -> System.out.println("Intaking Fuel")),
            intakeSubsystems.intake()
        ).withName("IntakeSequenceCommand");
    }

    public static Command endingIntakeSequence(IntakeSubsystems intakeSubsystems) {
        return Commands.sequence(
            Commands.runOnce(() -> System.out.println("Starting End Intake Sequence")),
            Commands.waitSeconds(0.5), // Change when we find the exact time it takes to do that action
            Commands.runOnce(() -> System.out.println("Stopping Intake Wheels")),
            intakeSubsystems.idle(),
            Commands.waitSeconds(0.5), // Change when we find the exact time it takes to do that action
            Commands.runOnce(() -> System.out.println("Lifting Intake To Travel Position")),
            intakeSubsystems.pivotToTravel()
        ).withName("EndIntakeSequenceCommand");
    }
}
