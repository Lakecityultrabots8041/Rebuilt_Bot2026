package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.ClimberSubsystem;

@SuppressWarnings("unused")

public class Climber extends Command {
    private final ClimberSubsystem climberSubsystem;

    public Climber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    public void execute() {
        System.out.println("running climber command");
        climberSubsystem.runClimber().execute();
    }
}