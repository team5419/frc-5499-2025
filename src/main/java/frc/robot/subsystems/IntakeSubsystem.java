package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem {
  private final CANSparkMax intake = new CANSparkMax(RobotMap.INTAKE, MotorType.kBrushless);

  public Command getDislogeCommand(int direction) {
    return this.runOnce(() -> intake.set(direction));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Disloger Subsystem/Disloger Motor Output", intake.get());
  }
}
