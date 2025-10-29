package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class SwerveDriveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();

    public final Pigeon2 pidgey = getPigeon2();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;

    private final VisionSubsystem visionSubsystem;

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

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
     *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
     *     theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public SwerveDriveSubsystem(
            VisionSubsystem visionSubsystem,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        this.visionSubsystem = visionSubsystem;

        // Set up the pose estimator
        modulePositionsSupplier = () -> {
            return Arrays.stream(getModules())
                    .map((SwerveModule<?, ?, ?> module) -> module.getCachedPosition())
                    .toArray(SwerveModulePosition[]::new);
        };

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            Logger.recordOutput("Odometry/Target Pose", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(),
                getPigeon2().getRotation2d(),
                modulePositionsSupplier.get(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return getState().Speeds;
    }

    public void configAutos() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            config = new RobotConfig(null, null, null, getModuleLocations());
        }

        PIDConstants translateConstants = new PIDConstants(10, 0.0, 0.0);
        PIDConstants rotationConstants = new PIDConstants(10, 0.0, 0.0);

        AutoBuilder.configure(
                () -> poseEstimator.getEstimatedPosition(),
                this::resetPose,
                this::getCurrentSpeeds,
                (speeds, feedforwards) -> this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)),
                new PPHolonomicDriveController(translateConstants, rotationConstants),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);

                m_hasAppliedOperatorPerspective = true;
            });
        }

        PoseEstimate limelightMeasurement = visionSubsystem.updateLimelight(pidgey);

        // If our angular velocity is greater than 360 degrees per second, ignore vision updates
        boolean doRejectUpdate = true;
        // if (Math.abs(pidgey.getAngularVelocityXDevice().getValueAsDouble()) > 360) {
        //   doRejectUpdate = true;
        // }
        if (limelightMeasurement != null && limelightMeasurement.tagCount == 0) {
            doRejectUpdate = false;
        }

        if (!doRejectUpdate) {
            // Add it to your pose estimator
            // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        }
        poseEstimator.update(pidgey.getRotation2d(), modulePositionsSupplier.get());

        Logger.recordOutput("Swerve Drive Subsystem/Chassis Speed", getCurrentSpeeds());
        Logger.recordOutput("Swerve Drive Subsystem/Swerve Pose", getPose());
        Logger.recordOutput("Swerve Drive Subsystem/Combined Pose", poseEstimator.getEstimatedPosition());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
