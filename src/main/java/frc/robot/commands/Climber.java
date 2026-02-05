package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class Climber extends Command {

    private final ClimberSubsystem climberSubsystem;
    private double CStep = 0;

    //==================Enumerator/case switch set up===============================================

    private enum step{
        ONE, TWO, THREE, FOUR, FIVE, ZERO
    }

    public static step uppie = step.ZERO;

   /* public static void one() {
        step uppie = step.ONE;
    }
    public static void two() {
        step uppie = step.TWO;
    }
    public static void three() {
        step uppie = step.THREE;
    }
    public static void four() {
        step uppie = step.FOUR;
    }
    public static void five() {
        step uppie = step.FIVE;
    }
    public static void zero() {
        step uppie = step.ZERO;
    } */

    //@Override
    public void periodic(){
        if (climberSubsystem.liftOver == true && uppie == step.ZERO) {
            step uppie = step.ONE;
        }
        if (climberSubsystem.upLock == true && uppie == step.ONE) {
            step uppie = step.TWO;
        }
        if (climberSubsystem.outOver == true && uppie == step.TWO) {
            step uppie = step.THREE;
        }
        if (climberSubsystem.outLock == true && uppie == step.THREE) {
            step uppie = step.FOUR;
        }
        //if(limelightSubsystem.onBar == true && uppie == step.FOUR) {
        // step uppie = step.FIVE;
        //}
        if (climberSubsystem.liftUnder == true && uppie == step.FIVE) {
            step uppie = step.ZERO;
        }
    }


   //================================Setup Done===================================================

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
        switch(uppie) {
            case ZERO:
            System.out.println("Climb started");
            climberSubsystem.liftServoRelease();
            withTimeout(0.5);
            System.out.println("Climber lift running");
            climberSubsystem.runLiftMotor(0.5);

            case ONE:
            System.out.println("Up done");
            climberSubsystem.stopHangMotors();
            climberSubsystem.liftServoLock();
            withTimeout(0.5);

            case TWO:
            System.out.println("RELEASE THE BABY");
            climberSubsystem.pivotServoRelease();
            withTimeout(0.5);
            climberSubsystem.runPivotMotor(0.5);

            case THREE:
            System.out.println("Pivoting done");
            climberSubsystem.stopHangMotors();
            climberSubsystem.pivotServoLock();
            withTimeout(0.5);

            case FOUR:
            System.out.println("Driving in my car, right after a beer");
            //Whatever we will have the limelight code for getting onto the bar be
            //TODO

            case FIVE:
            System.out.println("THE SKY IS FALLING");
            climberSubsystem.liftServoRelease();
            withTimeout(0.5);
            if (climberSubsystem.liftUnder = false) {
                climberSubsystem.runLiftMotor(-0.5);} else {
                    climberSubsystem.stopHangMotors();
                    climberSubsystem.liftServoLock();
                }

                

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
    
    

   private enum beans {
    L,
    M,
    H,
    HH
   }

   public static beans var = beans.L;

   public static void BM() {
    beans var = beans.M;
   }

   public static void BH() {
    beans var = beans.H;
   }

   public static void BHH() {
    beans var = beans.HH;
   }

   public static void BL() {
    beans var = beans.L;
   }

   public static void BeansF(beans[] args) {
    beans var = beans.L;

    switch(var) {
        case L:
        System.out.println("Low");
        BM();

        case M:
        System.out.println("Med");
        BH();

        case H:
        System.out.println("High");
        BHH();

        case HH:
        System.out.println("The Highest of Highs");
        BL();
    }
   }

  
   
    
}