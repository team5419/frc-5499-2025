import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;


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
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.DislogerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger
// the codey bit

CommandXboxController

joystick.x().whileTrue(drivetrain.applyRequest(() ->

          new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX())
        )
      )

public class IntakeSubsystem extends SubsystemBase {}

// Leo, make it so that when I press X, the motor spins backwards pleasel
// Everybot says that we need to have a climber forward and a climber backwards so maybe D pad
    @Override
  public void periodic() {
    Logger.recordOutput("Climb Subsystem/Left Encoder Position", .getPosition());
    Logger.recordOutput("Climb Subsystem/Right Encoder Position", .getPosition());
    Logger.recordOutput("Climb Subsystem/Left Encoder Velocity", .getVelocity());
    Logger.recordOutput("Climb Subsystem/Right Encoder Velocity", .getVelocity());
  }
