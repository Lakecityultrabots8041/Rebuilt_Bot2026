package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auton {
private final CommandSwerveDrivetrain drivetrain;
private final AutoFactory autoFactory;

    public Auton(
        AutoFactory autoFactory,
        CommandSwerveDrivetrain drivetrain
    ) {
        this.autoFactory = autoFactory;
        this.drivetrain = drivetrain;
    }

    /**
     * Simple autonomous routine that follows a single trajectory
     */
    public AutoRoutine simpleAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("SimpleAuto");
        
        // Load the trajectory - make sure this matches your path name
        final AutoTrajectory traj = routine.trajectory("SimpleAuto");

        // When auto starts, reset odometry and follow the trajectory
        routine.active().onTrue(
            traj.resetOdometry()
                .andThen(traj.cmd())
        );

        return routine;
    }

    /**
     * Do nothing autonomous - useful for testing or defense
     */
    public Command doNothingAuto() {
        return Commands.runOnce(() -> 
            System.out.println("Autonomous: Do Nothing")
        );
    }



    /**
     * This is an Example: Complex auto with multiple trajectories
     */
    /*public AutoRoutine complexAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("MultiplestepsAuto");
        final AutoTrajectory part1 = routine.trajectory("SimpleAuto");
        final AutoTrajectory part2 = routine.trajectory("MultiplestepsAuto.2");
        final AutoTrajectory part3 = routine.trajectory("MultiplestepsAuto.3");

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                part1.cmd(),
                
                // Add actions between trajectories (e.g., score, intake)
                Commands.waitSeconds(0.5),
                
                part2.cmd(),
                Commands.waitSeconds(0.5),
                
                part3.cmd()
            )
        );

        return routine;
    }
        */
}