package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignToReefCommand extends Command {
    private SwerveDriveSubsystem drivetrain;
    private PIDController xController, yController, rotController;
    private Timer dontSeeTagTimer, stopTimer;
    private double tagID = -1;

    public AlignToReefCommand(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        // xController
        // addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
