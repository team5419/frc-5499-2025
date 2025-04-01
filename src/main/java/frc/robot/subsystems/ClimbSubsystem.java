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


// Leo, make it so that when I press X, the motor spins backwards pleasel 
// - this gets done in robot container, NOT here

// Everybot says that we need to have a climber forward and a climber backwards so maybe D pad -- this will be in robot container NOT here
  @Override
  public void periodic() {
    Logger.recordOutput("Climb Motor Set Speed", climber.get());
    Logger.recordOutput("Climb Encoder Position", climbEncoder.getPosition());
    Logger.recordOutput("Climb Encoder Velocity", climbEncoder.getVelocity());

  }
}