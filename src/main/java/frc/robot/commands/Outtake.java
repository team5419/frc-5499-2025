package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Outtake extends Command {

    private IntakeSubsystem intake;
    private Timer timer;

    public Outtake(IntakeSubsystem intake) {
        this.intake = intake;
        timer = new Timer();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.setIntake(-1);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntake(0);
    }
}
