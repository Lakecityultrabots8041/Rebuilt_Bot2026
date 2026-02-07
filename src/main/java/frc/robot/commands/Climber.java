package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class Climber extends Command {

    private final ClimberSubsystem climberSubsystem;
    private ClimberSubsystem.Step lastStep = ClimberSubsystem.Step.ZERO;
    private double runTime = 0.0;

    public Climber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Climb sequence started - Direction: " + climberSubsystem.getClimbDirection());
    }

    @Override
    public void execute() {
        ClimberSubsystem.Step currentStep = climberSubsystem.getCurrentStep();
        
        // Reset timer when step changes
        if (currentStep != lastStep) {
            runTime = 0;
            lastStep = currentStep;
        }
        runTime += 0.02;
        
        // Determine motor direction based on climb direction and clumb state
        double direction;
        if (climberSubsystem.clumb == false) {
            direction = 0.5;
        } else {
            direction = -0.5;
        }
        
        switch(currentStep) {
            case ZERO:
                System.out.println("Climb started");
                climberSubsystem.liftServoRelease();
                if (runTime >= 0.5) {
                    System.out.println("Climber lift running");
                    climberSubsystem.runLiftMotor(direction);
                }
                break;

            case ONE:
                System.out.println("Up done");
                climberSubsystem.stopHangMotors();
                climberSubsystem.liftServoLock();
                break;

            case TWO:
                System.out.println("RELEASE THE BABY");
                climberSubsystem.pivotServoRelease();
                if (runTime >= 0.5) {
                    climberSubsystem.runPivotMotor(direction);
                }
                break;

            case THREE:
                System.out.println("Pivoting done");
                climberSubsystem.stopHangMotors();
                climberSubsystem.pivotServoLock();
                break;

            case FOUR:
                System.out.println("Driving in my car, right after a beer");
                //Whatever we will have the limelight code for getting onto the bar be
                //TODO
                break;

            case FIVE:
                System.out.println("THE SKY IS FALLING");
                climberSubsystem.liftServoRelease();
                if (runTime >= 0.5) {
                    if (climberSubsystem.liftUnder == false) {
                        climberSubsystem.runLiftMotor(-0.5);
                    } else {
                        climberSubsystem.stopHangMotors();
                        climberSubsystem.liftServoLock();
                    }
                }
                break;
        }
    }
}