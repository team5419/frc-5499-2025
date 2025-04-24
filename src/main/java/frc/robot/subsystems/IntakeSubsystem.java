package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkMax intake = new SparkMax(RobotMap.INTAKE, MotorType.kBrushless);
}

