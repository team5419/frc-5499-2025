package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final Pigeon2 gyro;
  private final SwerveDrivePoseEstimator poseEstimator;

  public Vision(Pigeon2 gyro, SwerveDrivePoseEstimator poseEstimator) {
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;
  }

  @Override
  public void periodic() {
    // First, tell Limelight your robot's current orientation
    double robotYaw = gyro.getYaw().getValueAsDouble();
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    // Add it to your pose estimator
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    poseEstimator.addVisionMeasurement(
      limelightMeasurement.pose,
      limelightMeasurement.timestampSeconds
    );
  }
}
