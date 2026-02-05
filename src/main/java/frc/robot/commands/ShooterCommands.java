package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {
    

    public static Command revUp(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::revCommand, shooter)
            .withName("SpinUpShooter");
    }

    public static Command shoot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::shootCommand, shooter)
            .withName("ShootingFuel");
    }

    public static Command idle(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::idleCommand, shooter)
            .withName("ShooterInIdle");
    }

    public static Command eject(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::shootCommand, shooter)
            .withName("ShooterEjecting");
    }

}
