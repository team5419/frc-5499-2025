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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.DislogerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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

  private final CommandXboxController controller = new CommandXboxController(0);

  private final SwerveDriveSubsystem drivetrain;
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
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
      drive
        .withVelocityX(-controller.getLeftY() * MaxSpeed)
        .withVelocityY(-controller.getLeftX() * MaxSpeed)
        .withRotationalRate(-controller.getRightX() * MaxAngularRate)
      )
    );

    // ---------- honestly i have no idea what pressing x does while driving ----------
    controller.x().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(
          new Rotation2d(-controller.getLeftY(), -controller.getLeftX())
        )
      )
    );

    controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // ---------- Intake ----------
    controller.rightTrigger().onTrue(
      intake.setIntakeCommand(1.0)
      .alongWith(Commands.runOnce(
        () -> controller.setRumble(RumbleType.kBothRumble, 0.5))
      )
    );
    controller.rightTrigger().onFalse(
      intake.setIntakeCommand(0)
      .alongWith(Commands.runOnce(
        () -> controller.setRumble(RumbleType.kBothRumble, 0.0))
      )
    );

    controller.leftTrigger().onTrue(intake.setIntakeCommand(-0.5));
    controller.leftTrigger().onFalse(intake.setIntakeCommand(0));

    // ---------- Elevator ----------
    controller.y().onTrue(elevator.changeElevateCommand(1));
    controller.a().onTrue(elevator.setElevateCommand(0));

    // ---------- Disloger ----------
    controller.leftBumper().onTrue(
      disloger.getDislogeCommand(1)
      .alongWith(Commands.runOnce(
        () -> controller.setRumble(RumbleType.kBothRumble, 0.5))
      )
    );
    controller.leftBumper().onFalse(
      disloger.getDislogeCommand(0)
      .alongWith(Commands.runOnce(
        () -> controller.setRumble(RumbleType.kBothRumble, 0.0))
      )
    );

    controller.rightBumper().onTrue(disloger.getDislogeCommand(-1));
    controller.rightBumper().onFalse(disloger.getDislogeCommand(0));

    // ---------- Reset heading ----------
    controller.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // ---------- Slowmode ----------
    controller.leftStick().onTrue(
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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
