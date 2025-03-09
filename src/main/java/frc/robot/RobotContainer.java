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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Disloger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

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

  private final CommandSwerveDrivetrain drivetrain;
  private final Elevator elevator;
  private final Disloger disloger;
  private final Lights lights;
  private final Intake intake;

  private boolean isSlowmode = false;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain = TunerConstants.createDrivetrain();
    elevator = new Elevator();
    disloger = new Disloger();
    lights = new Lights();
    intake = new Intake();

    NamedCommands.registerCommand("Elevator L1", elevator.setElevateCommand(0));
    NamedCommands.registerCommand("Elevator L2", elevator.setElevateCommand(1));
    NamedCommands.registerCommand("Elevator L3", elevator.setElevateCommand(2));
    NamedCommands.registerCommand("Disloger", disloger.getDislogeCommand(1));
    NamedCommands.registerCommand("Disloger Reverse", disloger.getDislogeCommand(-1));
    NamedCommands.registerCommand("Intake", intake.setIntakeCommand(1));
    NamedCommands.registerCommand("Intake Reverse", intake.setIntakeCommand(-1));

    autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // ---------- Driving ----------
    // TODO: Add deadband
    // Drivetrain will execute this command periodically
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // ---------- Intake ----------
    // joystick.rightBumper().onTrue(intake.setIntakeWithSensorCommand(0.25));
    // joystick.leftBumper().onTrue(intake.setIntakeCommand(-0.25));
    // joystick.rightBumper().onFalse(intake.setIntakeCommand(0));
    // joystick.leftBumper().onFalse(intake.setIntakeCommand(0));
    joystick.rightTrigger().onTrue(intake.setIntakeCommand(1.0));
    joystick.leftTrigger().onTrue(intake.setIntakeCommand(-0.5));
    joystick.rightTrigger().onFalse(intake.setIntakeCommand(0));
    joystick.leftTrigger().onFalse(intake.setIntakeCommand(0));

    // ---------- Elevator ----------
    // joystick.rightTrigger().onTrue(elevator.getElevateCommand(1));
    // joystick.rightTrigger().onFalse(elevator.getElevateCommand(0));
    // joystick.leftTrigger().onTrue(elevator.getElevateCommand(0.5));
    // joystick.leftTrigger().onFalse(elevator.getElevateCommand(0));
    joystick.y().onTrue(elevator.changeElevateCommand(1));
    joystick.a().onTrue(elevator.setElevateCommand(0));

    joystick.leftBumper().onTrue(disloger.getDislogeCommand(1));
    joystick.leftBumper().onFalse(disloger.getDislogeCommand(0));
    joystick.rightBumper().onTrue(disloger.getDislogeCommand(-1));
    joystick.rightBumper().onFalse(disloger.getDislogeCommand(0));

    // ---------- Disloger ----------

    // joystick.leftTrigger().onTrue(disloger.getDislogeCommand(1));
    // joystick.leftTrigger().onFalse(disloger.getDislogeCommand(0));

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
    // // return new PathPlannerPath("Test Path");
    // try {
    //   // Load the path you want to follow using its name in the GUI
    //   PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    //   // Create a path following command using AutoBuilder. This will also trigger event markers.
    //   return AutoBuilder.followPath(path);
    // } catch (Exception e) {
    //   DriverStation.reportError("error building auto: " + e.getMessage(), e.getStackTrace());
    //   return Commands.none();
    // }
  }
}
