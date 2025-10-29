package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagVisionConstants {
    public static final boolean kUsingVision = true;

    public static final double kCameraQuantity = 2;

    public static final String[] kCameraNames = {"OV9281_LEFT", "OV9281_RIGHT", "OV9782_3"};

    public static final String BACK_CAMERA_NAME = "OV9782_3";

    // RADIUM ROTATIONS/TRANSFORMS
    public static final Rotation3d[] kCameraRotations = null;

    public static final Transform3d[] kCameraTransform = null;

    public static final double kAmbiguityRejectionLimit = 0.25;
    public static final double kDistanceRejectionLimit = 3;
    public static final double kAreaMinimum = 0; // 0-100%

    public static final double kMaxVelocity = 5.4; // m / s
    public static final double kMaxRotationalVelocity = Math.PI; // rad / s - 45 degrees/sec

    public static final double kTranslationalStdDevMultiplier = 0.5;
    public static final double kRotationalStdDevMultiplier = 0.5; // Was 0.00125

    // used when there is a large delta in time between estimates
    public static final double kHighTrustTranslationalStdDevMultiplier = 0.08;
    public static final double kHighTrustRotationalStdDevMultiplier = 0.08;

    /** If the delta between vision readings is higher than this value, we start weighting readings higher after we get vision back */
    public static final double kMissingReadingsDelta = 1; // seconds
    /** The amount of time that we weight vision readings higher for after a large delta */
    public static final double kStopHighTrust = 0.2; // seconds

    public static final double kMinimumVisionStdDev = 0.02;
    public static final double kAverageDistanceStdDevMultiplier = 0.02;
}
