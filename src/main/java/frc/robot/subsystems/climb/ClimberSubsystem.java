package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.ClimberConstants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimberSubsystem extends SubsystemBase {

    private boolean ArmUp = true;
    private final TalonFX climbMotor;
    private boolean justClumb = false;
    private double timer = 0;

    public ClimberSubsystem() {
        climbMotor = new TalonFX(ClimberConstants.ClimbingMotor, ClimberConstants.CANIVORE_NAME);

        var CConfigs = new TalonFXConfiguration();
            CConfigs.Slot0.kP = ClimberConstants.Climb_KP;
            CConfigs.Slot0.kI = ClimberConstants.Climb_KI;
            CConfigs.Slot0.kD = ClimberConstants.Climb_KD;
        climbMotor.getConfigurator().apply(CConfigs);
    }


    public Command runClimber() {
        return Commands.run(() -> {
            if (ArmUp && justClumb == false) {
                climbMotor.set(-0.5);
                System.out.println("Climbing arm going down");
            } else if (justClumb == false) {
                climbMotor.set(0.5);
                System.out.println("Climbing arm going up");
            } else {
                climbMotor.set(0);
                System.out.println("just clumb, wait a second");
            }
        }, this);
    }

    @Override
    public void periodic() {
        if (justClumb == false){
        if (climbMotor.getPosition().getValueAsDouble() >= 0) {
            ArmUp = true;
            justClumb = true;
        }
        if (climbMotor.getPosition().getValueAsDouble() <= -500) {
            ArmUp = false;
            justClumb = true;
        }
    }
        if (justClumb == true) {
            timer += 0.02;
        }
        if (timer >= 1) {
            justClumb = false;
            timer = 0;
        }
    }
    

}