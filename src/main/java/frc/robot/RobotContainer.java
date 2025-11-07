// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DislogerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LightsState;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhoton;
// import frc.robot.RobotState; // Uncomment this line if RobotState exists in the package

public class RobotContainer {

    AprilTagVision aprilTagVision;

    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(0);

    private final SwerveDriveSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final DislogerSubsystem disloger;
    private final LightsSubsystem lights;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;
    private final ClimbSubsystem climb;
    private boolean aligning = false;
    private boolean isSlowmode = false;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        lights = new LightsSubsystem();

        elevator = new ElevatorSubsystem(lights);
        disloger = new DislogerSubsystem();
        intake = new IntakeSubsystem();
        climb = new ClimbSubsystem();
        vision = new VisionSubsystem();
        drivetrain = TunerConstants.createDrivetrain(vision);
        aprilTagVision = new AprilTagVision(this, new AprilTagVisionIOPhoton());

        drivetrain.configAutos();

        lights.setState(LightsState.DISABLED);

        NamedCommands.registerCommand("Elevator L1", elevator.setElevateCommand(0));
        NamedCommands.registerCommand("Elevator L2", elevator.setElevateCommand(1));
        NamedCommands.registerCommand("Elevator L3", elevator.setElevateCommand(2));
        NamedCommands.registerCommand("Disloger", disloger.getDislogeCommand(1));
        NamedCommands.registerCommand("Disloger Stop", disloger.getDislogeCommand(0));
        NamedCommands.registerCommand("Disloger Reverse", disloger.getDislogeCommand(-1));
        NamedCommands.registerCommand("Intake", intake.setIntakeCommand(1));
        NamedCommands.registerCommand("Outtake", intake.setIntakeCommand(-1));
        NamedCommands.registerCommand("Intake Stop", intake.setIntakeCommand(0));

        autoChooser = AutoBuilder.buildAutoChooser("coral");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    public SwerveDriveSubsystem getSwerve() {
        return drivetrain;
    }

    public boolean isAligning() {
        return aligning;
    }

    private void configureBindings() {
        // ---------- Driving ----------
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // driver controls
        // left bumper: slowmode
        // left trigger: readjust coral
        // right bumper: auto align
        // right trigger: intake/outtake
        // b: climb
        // y: gyro reset
        // a: stow elevator

        // slowmode
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> {
            isSlowmode = !isSlowmode;
            if (isSlowmode) {
                MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4;
                MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) / 2;
            } else {
                MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond);
            }
        }));
        //Left Bumper - Auto Align
        //Shoot Manual
        driver.rightTrigger().onTrue(intake.setIntakeCommand(-0.5));
        driver.rightTrigger().onFalse(intake.setIntakeCommand(0));

        // Dislodge. 
        driver.leftTrigger().onTrue(intake.setIntakeCommand(1.0));
        driver.leftTrigger().onFalse(intake.setIntakeCommand(0));

        // elevator up
        driver.x().onTrue(elevator.changeElevateCommand(1));

        // stow elevator
        driver.a().onTrue(elevator.setElevateCommand(0));

        // climb in
        driver.b().onTrue(climb.setClimberCommand(-.5));
        driver.b().onFalse(climb.setClimberCommand(0));

        // gyro reset
        driver.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // operator controls
        // pov up: L3
        // pov left: L2
        // pov down: L1
        // start: emergency something
        // left bumper: align late
        // right bumper: align early
        // right trigger: dislodge
        // unclimb: x?

        // unclimb
        operator.x().onTrue(climb.setClimberCommand(.5));
        operator.x().onFalse(climb.setClimberCommand(0));
        //Triggers - Intake
        //Bumper, Early Late
        //D pad, 2-3
        // dislodge
        //X,A Climb
        operator.rightTrigger().onTrue(disloger.getDislogeCommand(1));
        operator.rightTrigger().onFalse(disloger.getDislogeCommand(0));
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
                "Intake Coral", new InstantCommand());
        NamedCommands.registerCommand(
                "Score Coral", new InstantCommand());
        NamedCommands.registerCommand(
                "Dealgea", new InstantCommand());
        NamedCommands.registerCommand(
                "Auto Align", new InstantCommand());
    }
    public Command getAutonomousCommand() {
        System.out.println(autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }
}
