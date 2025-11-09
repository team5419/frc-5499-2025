// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlignToCoral;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Outtake;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DislogerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorGoal;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LightsState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhoton;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.generated.TunerConstantsBearium;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;
import java.io.File;
import lombok.Getter;

public class RobotContainer {

    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstantsBearium.kSpeedAt12Volts.in(MetersPerSecond);
    // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Setting up bindings for necessary control of the swerve drive platform
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1)
    //         .withRotationalDeadband(MaxAngularRate * 0.1)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final AprilTagVision aprilTagVision;

    @Getter
    private final Swerve swerve;

    // private final SwerveDriveSubsystem drivetrain;

    @Getter
    private final ElevatorSubsystem elevator;

    @Getter
    private final DislogerSubsystem disloger;

    @Getter
    private final LightsSubsystem lights;

    @Getter
    private final IntakeSubsystem intake;

    @Getter
    private final ClimbSubsystem climb;

    private boolean aligning = false;

    public boolean isAligning() {
        return aligning;
    }

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        lights = new LightsSubsystem();

        elevator = new ElevatorSubsystem();
        disloger = new DislogerSubsystem();
        intake = new IntakeSubsystem();
        climb = new ClimbSubsystem();
        // vision = new VisionSubsystem();
        // drivetrain = TunerConstants.createDrivetrain(vision);
        aprilTagVision = new AprilTagVision(this, new AprilTagVisionIOPhoton());
        swerve = new Swerve(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontLeft()),
                new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontRight()),
                new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackLeft()),
                new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackRight()));
        // drivetrain.configAutos();

        lights.setState(LightsState.DISABLED);

        autoChooser = AutoBuilder.buildAutoChooser("coral");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configNamedCommands();
        configureBindings();
    }

    private void configureBindings() {
        // ---------- Driving ----------
        swerve.setDefaultCommand(DriveCommands.joystickDrive(
                swerve,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> driver.leftBumper().getAsBoolean()));

        // driver controls
        // left bumper: slowmode
        // left trigger: readjust coral
        // right bumper: auto align
        // right trigger: intake/outtake
        // b: climb
        // y: gyro reset
        // a: stow elevator

        // slowmode
        driver.leftBumper();

        // stow
        driver.a().onTrue(elevator.setElevateCommand(ElevatorGoal.STOW));

        // readjust
        driver.leftTrigger().onTrue(intake.setIntakeCommand(0.5));
        driver.leftTrigger().onFalse(intake.setIntakeCommand(0));

        // auto align
        // driver.rightBumper().onTrue(new AutoAlignToCoral(this, driver));
        // driver.b().onTrue(elevator.setElevateCommand());
        // driver.b().onFalse(elevator.setElevateCommand())

        // outtake
        driver.rightTrigger().onTrue(elevator.setElevateCommand()).onFalse(new Outtake(intake));
        driver.rightBumper()
                .onTrue(new ParallelCommandGroup(elevator.setElevateCommand(), new AutoAlignToCoral(this, driver)))
                .onFalse(new Outtake(intake));

        // unclimb
        driver.povDown().onTrue(climb.setClimberCommand(.5));
        driver.povDown().onFalse(climb.setClimberCommand(0));

        // gyro reset
        driver.y().onTrue(Commands.runOnce(() -> swerve.resetGyro()).ignoringDisable(true));

        // operator controls
        // L2-L3
        operator.povRight().onTrue(new InstantCommand(() -> elevator.setCurrentGoal(ElevatorGoal.L3)));
        operator.povLeft().onTrue(new InstantCommand(() -> elevator.setCurrentGoal(ElevatorGoal.L2)));
        operator.povDown().onTrue(new InstantCommand(() -> elevator.setCurrentGoal(ElevatorGoal.L1)));

        // climb in
        operator.b().onTrue(climb.setClimberCommand(-.5));
        operator.b().onFalse(climb.setClimberCommand(0));

        // unclimb
        operator.x().onTrue(climb.setClimberCommand(.5));
        operator.x().onFalse(climb.setClimberCommand(0));
        // Trigger - Intake
        operator.leftTrigger().onTrue(intake.setIntakeWithSensorCommand(-0.5));
        operator.leftTrigger().onFalse(intake.setIntakeWithSensorCommand(0));

        operator.rightTrigger().onTrue(intake.setIntakeWithSensorCommand(0.5));
        operator.rightTrigger().onFalse(intake.setIntakeWithSensorCommand(0));
        // Dislodge
        operator.a().onTrue(new ParallelCommandGroup(disloger.getDislogeCommand(1), elevator.setElevateCommand()));
        operator.a().onFalse(new ParallelCommandGroup(disloger.getDislogeCommand(0), elevator.setElevateCommand()));
        // Deploy Dislodger
        operator.y().onTrue(disloger.getDislogeCommand(-1));
        operator.y().onFalse(disloger.getDislogeCommand(0));
        // Early - Late
        operator.leftBumper()
                .onTrue(new InstantCommand(() -> RobotState.getInstance().setEarly(false)));
        operator.rightBumper()
                .onTrue(new InstantCommand(() -> RobotState.getInstance().setEarly(true)));
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the
    // "tx" value from the Limelight.
    double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging
    // rather than "ty"
    double limelight_range_proportional() {
        double kP = 0.1;
        double targetingForwardSpeed = LimelightHelpers.getTY("") * kP;
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    private void configNamedCommands() {
        NamedCommands.registerCommand(
                "Record Time", new InstantCommand(() -> RobotState.getInstance().setAutoFinished(true)));
        NamedCommands.registerCommand(
                "Intake Coral", new ParallelRaceGroup(intake.setIntakeCommand(-0.2), new WaitCommand(0.5)));
        NamedCommands.registerCommand(
                "Score Coral",
                new SequentialCommandGroup(
                        intake.setIntakeCommand(-0.5),
                        new WaitCommand(0.3),
                        new ParallelCommandGroup(
                                new RunCommand(() -> elevator.updateElevator(ElevatorGoal.STOW)),
                                intake.setIntakeCommand(0))));
        NamedCommands.registerCommand(
                "Dealgea",
                new ParallelCommandGroup(
                        disloger.getDislogeCommand(1), new RunCommand(() -> elevator.updateElevator(ElevatorGoal.L2))));
        NamedCommands.registerCommand("Auto Align", new AutoAlignToCoral(this, driver));
    }

    public Command getAutonomousCommand() {
        System.out.println(autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }
    ;

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do nothing", new InstantCommand());
        File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        for (File f : autosDir.listFiles()) {
            if (!f.isDirectory()) {
                String fileName[] = f.getName().split("\\.");
                String autoName = fileName[0];
                chooser.addOption(
                        autoName, AutoBuilder.buildAuto(autoName).beforeStarting(() -> System.out.println(autoName)));
            }
        }
        chooser.addOption("3-Coral-Right", new PathPlannerAuto("3-Coral-Left", true));
        return chooser;
    }

    public AprilTagVision getAprilTagVision() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAprilTagVision'");
    }
}
