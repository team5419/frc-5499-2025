package frc.robot;

import lombok.Getter;
import lombok.Setter;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    @Getter
    @Setter
    private boolean isEarly;

    @Getter
    @Setter
    private boolean autoAlignAtGoal;

    @Getter
    @Setter
    private boolean isAligning;
}
