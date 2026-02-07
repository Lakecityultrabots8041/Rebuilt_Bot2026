package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class Climber extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final ClimberSubsystem.step uppie;
    private ClimberSubsystem.step last = ClimberSubsystem.step.ZERO;
    private double direction = 0.5;

    private double runTime = 0.0;


   //================================Setup Done===================================================

    public Climber(ClimberSubsystem climberSubsystem, ClimberSubsystem.step uppie) {
        this.climberSubsystem = climberSubsystem;
        this.uppie = uppie;
        

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Climb sequence started");
    }


    @Override
    public void execute() {
        if (uppie != last){
            runTime = 0;
            last = uppie;
        }
        runTime += 0.02;
         
        switch(uppie) {
            case ZERO:
            System.out.println("Testing Motor");
            climberSubsystem.runLiftMotor(1);
            /*System.out.println("Climb started");
            climberSubsystem.liftServoRelease();
            //if (runTime >= 0.5) {
            System.out.println("Climber lift running");
            climberSubsystem.runLiftMotor(direction);//}*/
            break;

            case ONE:
            System.out.println("Up done");
            climberSubsystem.stopHangMotors();
            climberSubsystem.liftServoLock();
            break;

            case TWO:
            System.out.println("RELEASE THE BABY");
            climberSubsystem.pivotServoRelease();
            if (runTime >= 1.5){
            climberSubsystem.runPivotMotor(direction);}
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
            if (runTime >= 0.5){
            if (climberSubsystem.liftUnder == false) {
                climberSubsystem.runLiftMotor(-0.5);} else {
                    climberSubsystem.stopHangMotors();
                    climberSubsystem.liftServoLock();
                    
                }}
            break;

            case CLUMB:
            if(direction == 0.5){
                direction = -0.5;
            } else {
                direction = 0.5;
            }

            break;

                

        }
    
    }

    


   /*  public void execute() {
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
    }   */
    
    

   
  
   
    
}