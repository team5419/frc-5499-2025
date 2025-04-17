package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSystem extends SubsystemBase{
    private final SparkMax intake = new SparkMax(RobotMap.INTAKE, MotorType.kBrushless);
}

public class IntakeSubsystem {
    
}


