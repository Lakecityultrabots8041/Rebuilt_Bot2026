package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.robot.Constants;
import frc.robot.Constants.ShooterCostants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@SuppressWarnings("unused")

public class ShooterSubsystem extends SubsystemBase {
    
    // speed presets
    private static final double maxVel = 100;  // change once we find out the max velocity
    private static final double revVel = 12.5; // 1/8 of maxVel(Change when we find maxVel)
    private static final int eject = -5; //The speed of the motors ejecting a ball if stuck

    private final TalonFX shootMotor;

    private shooterState currentState = shooterState.IDLE;

    public enum shooterState {
        IDLE,
        REVUP,
        READY,
        EJECTING
    }

public ShooterSubsystem(){

    shootMotor = new TalonFX(Constants.ShooterCostants.SHOOTER_MOTOR);

}

     // Control Modes(Change Later)
    VelocityTorqueCurrentFOC torqueRequest = new VelocityTorqueCurrentFOC(0);

    //Methods
    public void maxSpeed() {
        shootMotor.setControl(torqueRequest.withVelocity(maxVel));
    }

    public void revSpeed() {
        shootMotor.setControl(torqueRequest.withVelocity(revVel));
    }

    public void stopShooting() {
        shootMotor.setControl(torqueRequest.withVelocity(0));
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case IDLE:
                shootMotor.setControl(torqueRequest.withVelocity(0));
                break;

            case REVUP:
                shootMotor.setControl(torqueRequest.withVelocity(maxVel));
                Commands.waitSeconds(1);
                break;

            case READY:
                shootMotor.setControl(torqueRequest.withVelocity(maxVel)); 
                break;
                
            case EJECTING:
                shootMotor.setControl(torqueRequest.withVelocity(maxVel));
                break;
        }
    }

    //Command Factories
    public Command revCommand() {
        return runOnce(() -> currentState = shooterState.REVUP)
                .withName("ShooterReving");
    }

    public Command shootCommand() {
        return runOnce(() -> currentState = shooterState.READY)
                .withName("ShooterReady");
    }

    public Command ejectCommand() {
        return runOnce(() -> currentState = shooterState.EJECTING)
                .withName("ShooterEject");
    }

    public Command idleCommand() {
        return runOnce(() -> currentState = shooterState.IDLE)
                .withName("ShooterIdle");
    }




}
