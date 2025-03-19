package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final Pigeon2 gyro;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state
   * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
   * then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
   * and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
    this.gyro = drivetrain.getPigeon2();

    // Set up the pose estimator
    modulePositionsSupplier = () -> {
      return Arrays.stream(drivetrain.getModules())
        .map((SwerveModule<?, ?, ?> module) -> module.getCachedPosition())
        .toArray(SwerveModulePosition[]::new);
    };

    poseEstimator = new SwerveDrivePoseEstimator(
      drivetrain.getKinematics(),
      drivetrain.getPigeon2().getRotation2d(),
      modulePositionsSupplier.get(),
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
  }

  @Override
  public void periodic() {
    // First, tell Limelight your robot's current orientation
    // double robotYaw = gyro.getYaw().getValueAsDouble();
    double robotYaw = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation("limelight", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // If our angular velocity is greater than 360 degrees per second, ignore vision updates
    boolean doRejectUpdate = false;
    if (Math.abs(gyro.getAngularVelocityXDevice().getValueAsDouble()) > 360) {
      doRejectUpdate = true;
    }
    if (limelightMeasurement.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      // Add it to your pose estimator
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
      poseEstimator.addVisionMeasurement(
        limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }

    poseEstimator.update(
      new Rotation2d(gyro.getYaw().getValueAsDouble()),
      modulePositionsSupplier.get()
    );

    Logger.recordOutput("Vision Subsystem/Pose Estimate", poseEstimator.getEstimatedPosition());
  }
}
