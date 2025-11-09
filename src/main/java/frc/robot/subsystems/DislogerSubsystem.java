package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class DislogerSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(RobotMap.DISLOGER, MotorType.kBrushless);

    public Command getDislogeCommand(int direction) {
        Logger.recordOutput("Dislodger Subsystem/Dislodger Requested Direction", direction);
        return this.runOnce(() -> motor.set(direction));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Dislodger Subsystem/Dislodger Motor Output", motor.get());
    }
}
