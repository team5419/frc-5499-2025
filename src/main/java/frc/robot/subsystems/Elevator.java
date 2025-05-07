package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase{
        private final SparkMax leftElevator = new SparkMax (RobotMap.ELEVATOR_LEFT, MotorType.kBrushless);
        private final SparkMax rightElevator = new SparkMax (RobotMap.ELEVATOR_RIGHT, MotorType.kBrushless);

        private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        private final SparkClosedLoopController rightController = rightElevator.getClosedLoopController();
        private final SparkClosedLoopController lefController = leftElevator.getClosedLoopController(); 

        private final RelativeEncoder leftEncoder = leftElevator.getEncoder(); 
        private final RelativeEncoder rightEncoder = rightElevator.getEncoder();

        private int currentPosition = 0;
    

    
}
