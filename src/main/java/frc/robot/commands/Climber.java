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
        System.out.println("Climb sequence started");
    }

    @Override
    public void execute() {
        // Read the current step from the subsystem
        ClimberSubsystem.Step currentStep = climberSubsystem.getCurrentStep();
        
        // Reset timer when step changes
        if (currentStep != lastStep) {
            runTime = 0;
            lastStep = currentStep;
        }
        runTime += 0.02;
        
        // Direction based on whether we're climbing up or down
        double direction = climberSubsystem.climbingUp ? 0.6 : -0.6;
         
        switch(currentStep) {
            case ZERO:
                // For testing: just run motor at full speed
                System.out.println("Testing Motor - Running at full speed");
                climberSubsystem.runLiftMotor(1.0);
                
                /* Full climb sequence (uncomment when ready):
                System.out.println("Climb started");
                climberSubsystem.liftServoRelease();
                if (runTime >= 0.5) {
                    System.out.println("Climber lift running");
                    climberSubsystem.runLiftMotor(direction);
                }
                */
                break;

            case ONE:
                System.out.println("Lift complete");
                climberSubsystem.stopHangMotors();
                climberSubsystem.liftServoLock();
                break;

            case TWO:
                System.out.println("Starting pivot");
                climberSubsystem.pivotServoRelease();
                if (runTime >= 0.5) {
                    climberSubsystem.runPivotMotor(direction);
                }
                break;

            case THREE:
                System.out.println("Pivot complete");
                climberSubsystem.stopHangMotors();
                climberSubsystem.pivotServoLock();
                break;

            case FOUR:
                System.out.println("Awaiting bar alignment");
                // Limelight code here
                break;

            case FIVE:
                System.out.println("Final descent/ascent");
                climberSubsystem.liftServoRelease();
                if (runTime >= 0.5) {
                    if (climberSubsystem.liftUnder == false) {
                        climberSubsystem.runLiftMotor(climberSubsystem.climbingUp ? -0.5 : 0.5);
                    } else {
                        climberSubsystem.stopHangMotors();
                        climberSubsystem.liftServoLock();
                    }
                }
                break;

            case CLUMB:
                System.out.println("Direction toggled - resetting to ZERO");
                // Direction toggle happens in periodic(), just acknowledge it here
                break;
        }
    }

    @Override
    public boolean isFinished() {
    // End when we reach Step.CLUMB (full sequence complete)
    return climberSubsystem.getCurrentStep() == ClimberSubsystem.Step.CLUMB;
    }

    @Override
    public void end(boolean interrupted) {
    climberSubsystem.stopHangMotors();
    if (interrupted) {
        System.out.println("Climb sequence interrupted");
    } else {
        System.out.println("Climb sequence complete");
    }
    }
}