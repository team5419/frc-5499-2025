// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.DislogerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LightsSubsystem.LightsState;

public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final SwerveDriveSubsystem drivetrain;
  private final ElevatorSubsystem elevator;
  private final DislogerSubsystem disloger;
  private final LightsSubsystem lights;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;

  private boolean isSlowmode = false;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    lights = new LightsSubsystem();

    drivetrain = TunerConstants.createDrivetrain();
    elevator = new ElevatorSubsystem(lights);
    disloger = new DislogerSubsystem();
    intake = new IntakeSubsystem();
    vision = new VisionSubsystem(drivetrain);

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

    autoChooser = AutoBuilder.buildAutoChooser("Move Forward Short");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // ---------- Driving ----------
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
      drive
        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
      )
    );

    // // ---------- honestly i have no idea what pressing x does while driving ----------
    // joystick.x().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(
    //       new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX())
    //     )
    //   )
    // );

    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // ---------- Intake ----------
    joystick.rightTrigger().onTrue(intake.setIntakeCommand(1.0));
    joystick.leftTrigger().onTrue(intake.setIntakeCommand(-0.5));
    joystick.rightTrigger().onFalse(intake.setIntakeCommand(0));
    joystick.leftTrigger().onFalse(intake.setIntakeCommand(0));

    // ---------- Elevator ----------
    joystick.y().onTrue(elevator.changeElevateCommand(1));
    joystick.a().onTrue(elevator.setElevateCommand(0));

    // ---------- Disloger ----------
    joystick.leftBumper().onTrue(disloger.getDislogeCommand(1));
    joystick.leftBumper().onFalse(disloger.getDislogeCommand(0));
    joystick.rightBumper().onTrue(disloger.getDislogeCommand(-1));
    joystick.rightBumper().onFalse(disloger.getDislogeCommand(0));

    // ---------- Reset heading ----------
    joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // ---------- Slowmode ----------
    joystick.leftStick().onTrue(
      drivetrain.runOnce(() -> {
        isSlowmode = !isSlowmode;
        if (isSlowmode) {
          MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4;
          MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) / 2;
        } else {
          MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
          MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond);
        }
      })
    );
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
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional() {
    double kP = 0.1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
