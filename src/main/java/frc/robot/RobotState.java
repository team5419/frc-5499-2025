package frc.robot;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    @Getter
    @Setter
    @AutoLogOutput(key = "RobotState/Is early")
    private boolean isEarly;

    @Getter
    @Setter
    @AutoLogOutput(key = "RobotState/Auto align at goal")
    private boolean autoAlignAtGoal;

    @Getter
    @Setter
    @AutoLogOutput(key = "RobotState/Is aligning")
    private boolean isAligning;

    @Getter
    @Setter
    private boolean isAutoFinished;
}
