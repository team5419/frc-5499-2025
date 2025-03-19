// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrainSubsystem;
import frc.robot.subsystems.DislogerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final CommandSwerveDrivetrainSubsystem drivetrain;
  private final ElevatorSubsystem elevator;
  private final DislogerSubsystem disloger;
  private final LightsSubsystem lights;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;

  private boolean isSlowmode = false;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain = TunerConstants.createDrivetrain();
    elevator = new ElevatorSubsystem();
    disloger = new DislogerSubsystem();
    lights = new LightsSubsystem();
    intake = new IntakeSubsystem();
    vision = new VisionSubsystem(drivetrain);

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
    // TODO: Add controller deadbanding
    // Drivetrain will execute this command periodically
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    joystick
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

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
    joystick
        .leftStick()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  isSlowmode = !isSlowmode;
                  if (isSlowmode) {
                    MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4;
                    MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) / 2;
                  } else {
                    MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                    MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond);
                  }
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
