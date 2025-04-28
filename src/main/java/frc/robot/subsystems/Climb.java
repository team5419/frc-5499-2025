package frc.robot.subsystems;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//imports the code from the other subsytems
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

//adds what SparkMax and MotorType is
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//Creates a class that links to the robot map, so that when you iniciate the command in robotmap, the new motor spins (double(Which is a number variable like boolen))
//adds a new motor to the system that is from the same id as in CLIMBER in the robot map, and is a brushless type
public class Climb extends SubsystemBase {
    private final SparkMax climb = new SparkMax(RobotMap.CLIMBER, MotorType.kBrushless);
    public Command setClimbCommand(double direction)
    return Commands.runOnce(() -> climb.set(direction));
}