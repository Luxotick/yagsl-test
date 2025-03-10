package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to continuously rotate the shooter while a button is held.
 */
public class ShooterPositionCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final boolean isRotatingUp;

    /**
     * Creates a new ShooterPositionCommand.
     * 
     * @param shooterSubsystem The shooter subsystem to control
     * @param isRotatingUp True to rotate up, false to rotate down
     */
    public ShooterPositionCommand(ShooterSubsystem shooterSubsystem, boolean isRotatingUp) {
        this.shooterSubsystem = shooterSubsystem;
        this.isRotatingUp = isRotatingUp;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // Start rotation immediately
        if (isRotatingUp) {
            shooterSubsystem.rotateUp();
        } else {
            shooterSubsystem.rotateDown();
        }
    }

    @Override
    public void execute() {
        // Continue rotation in case it was stopped by a limit
        if (isRotatingUp) {
            shooterSubsystem.rotateUp();
        } else {
            shooterSubsystem.rotateDown();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop rotation when the command ends
        shooterSubsystem.stopRotation();
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted (button released)
        return false;
    }
} 