package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class Climber extends Command {

    private final ClimberSubsystem climberSubsystem;
    private double CStep = 0;

   

    public Climber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override

    public void initialize() {
        System.out.println("Climb sequence started");
    }

    @Override

    public void execute() {
        if (CStep == 0) {
            System.out.println("Climb started");
            climberSubsystem.liftServoRelease();
            withTimeout(0.5);
            CStep = 1;
        }
        if (CStep == 1) {
            System.out.println("Erecting the climber");
            climberSubsystem.runLiftMotor(0.5);
        }
        if (climberSubsystem.liftOver == true && CStep == 1) {
            System.out.println("Up done");
            climberSubsystem.liftServoLock();
            CStep = 2;
        }
        if (CStep == 2) {
            System.out.println("RELEASE THE BABY");
            climberSubsystem.pivotServoRelease();
            withTimeout(0.5);
            CStep = 3;
        }
        if (CStep == 3) {
            System.out.println("The baby is on the move");
            climberSubsystem.runPivotMotor(0.5);
        }
        if (climberSubsystem.outOver == true && CStep == 3) {
            System.out.println("Out done");
            climberSubsystem.pivotServoLock();
            CStep = 4;
        }
    }   

   

    
}