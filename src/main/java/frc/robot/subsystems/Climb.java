package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final SparkMax climb = new SparkMax(RobotMap.CLIMB, MotorType.kBrushless);

  public Command setClimbCommand(double direction) {
    return Commands.runOnce(() -> climb.set(direction));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climb Motor Output", climb.get());
  }
}
