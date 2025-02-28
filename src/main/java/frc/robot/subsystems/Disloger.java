package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Disloger extends SubsystemBase {
  private final SparkMax motor = new SparkMax(RobotMap.DISLOGER, MotorType.kBrushless);

  // public Command getDislogeCommand(int direction)
}
