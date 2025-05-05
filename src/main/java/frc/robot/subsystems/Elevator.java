package frc.robot.subsystems;
public class Elevator {
    public class Elevator extends SubsystemBase{
        private final SparkMax leftElevator = new SparkMax (RobotMap.LEFT_ELEVATOR, MotorType.kBrushless);
        private final SparkMax rightElevator = new SparkMax (RobotMap.RIGHT_ELEVATOR, MotorType.kBrushless);

        private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        private final SparkClosedLoopController leftController = leftElevator.getClosedLoopController();
           
    }
}
