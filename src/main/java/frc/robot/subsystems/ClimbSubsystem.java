package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax climber = new SparkMax(RobotMap.CLIMBER, MotorType.kBrushless);

  private final RelativeEncoder climbEncoder = climber.getEncoder();

  public Command setClimberCommand(double direction) {
    return Commands.runOnce(() -> climber.set(direction));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climb Subsystem/Speed", climber.get());
    Logger.recordOutput("Climb Subsystem/Encoder Position", climbEncoder.getPosition());
    Logger.recordOutput("Climb Subsystem/Encoder Velocity", climbEncoder.getVelocity());
  }
}
