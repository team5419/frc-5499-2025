package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem {
  private final SparkMax intake = new SparkMax(RobotMap.INTAKE, MotorType.kBrushless);

  public Command setIntakeCommand(double direction) {
    return Commands.runOnce(() -> intake.set(direction));
    
  }
  /* 
  @Override
  public void periodic() {
    Logger.recordOutput("Disloger Subsystem/Disloger Motor Output", intake.get());
  }
*/
}
