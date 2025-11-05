package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.apriltagvision.AprilTagVision;

public class AutoScore extends SequentialCommandGroup {

    private AprilTagVision vision;

    public AutoScore(RobotContainer robot) {
        vision = robot.getAprilTagVision();

        addCommands();
    }
}
