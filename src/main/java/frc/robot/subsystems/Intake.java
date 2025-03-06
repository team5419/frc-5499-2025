package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private final SparkMax intake = new SparkMax(RobotMap.INTAKE, MotorType.kBrushless);
  DigitalInput input = new DigitalInput(RobotMap.BEAM_BREAK); // make channel 9 a constant

  public Intake() {}

  public Command setIntakeCommand(double direction) {
    return Commands.runOnce(() -> intake.set(direction));
  }

  public Command setIntakeWithSensorCommand(double direction) {
    return Commands.runOnce(
        () -> {
          if (input.get()) {
            intake.set(0);
          } else {
            intake.set(direction);
          }
        });
  }

  @Override
  public void periodic() {
    // System.out.println(input.get());
  }

  public boolean getBeamBreak() {
    return input.get();
  }
}
