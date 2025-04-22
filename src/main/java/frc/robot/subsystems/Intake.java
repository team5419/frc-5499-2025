package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    public Intake () {}

    private final SparkMax intake = new SparkMax (RobotMap.INTAKE,MotorType.kBrushless);

    public Command setIntakeCommand (double direction) {
        return Commands.runOnce(() -> intake.set (direction));
    }

    @Override
    public void periodic (){
        Logger.recordOutput ("Intake Motor Output", intake.get());
    }
}


