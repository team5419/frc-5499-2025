package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
public Intake () {}
public class Intake extends SubsystemBase {
private final SparkMax intake = new SparkMax (RobotMap.INTAKE,MotorType.kBrushless);

public Command setIntakeCommand (double direction); {
return Commands.runOnce(() -> intake.set (direction));

}

}


@Override
public void periodic ()
Logger.recordOutput (key: "Intake Motor Output", intake.get());
}

