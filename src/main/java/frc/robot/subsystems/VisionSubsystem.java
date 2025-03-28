package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  public PoseEstimate updateLimelight(Pigeon2 pidgey) {
    // First, tell Limelight your robot's current orientation
    double robotYaw = pidgey.getYaw().getValueAsDouble();
    // double robotYaw = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    Logger.recordOutput("Vision Subsystem/Limelight Pose", limelightMeasurement.pose);
    Logger.recordOutput("Vision Subsystem/TX", LimelightHelpers.getTX(""));
    Logger.recordOutput("Vision Subsystem/TY", LimelightHelpers.getTY(""));
    Logger.recordOutput("Vision Subsystem/Tag Count", limelightMeasurement.tagCount);

    return limelightMeasurement;
  }
}
