package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    public static final double[] elevatorPositions = /*{0, 95, 134}*/ {0, 3.5, 14.6};
    public static final double elevatorConversion = /*0.0368421053*/ 1;

    private final SparkMax leftElevator = new SparkMax(RobotMap.LEFT_ELEVATOR, MotorType.kBrushless);
    private final SparkMax rightElevator = new SparkMax(RobotMap.RIGHT_ELEVATOR, MotorType.kBrushless);

    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    private final SparkClosedLoopController leftController = leftElevator.getClosedLoopController();
    private final SparkClosedLoopController rightController = rightElevator.getClosedLoopController();

    private final RelativeEncoder leftEncoder = leftElevator.getEncoder();
    private final RelativeEncoder rightEncoder = rightElevator.getEncoder();

    private static final LoggedTunableNumber stow = new LoggedTunableNumber("Elevator/Stow Height", 0);
    private static final LoggedTunableNumber l2 = new LoggedTunableNumber("Elevator/L2", 1.6);
    private static final LoggedTunableNumber l3 = new LoggedTunableNumber("Elevator/L3", 3.4);

    public enum ElevatorGoal {
        IDLE(() -> 0), // Should be the current height
        STOW(stow),
        INTAKE_FAR(stow),
        L1(stow),
        L2(l2),
        L3(l3);

        @Getter
        private DoubleSupplier eleHeight;

        private ElevatorGoal(DoubleSupplier eleHeight) {
            this.eleHeight = eleHeight;
        }
    }

    private int currentPosition = 0;

    public ElevatorSubsystem() {

        elevatorConfig.closedLoop.p(0.15).outputRange(-1, 1);

        rightElevator.configure(
                elevatorConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftElevator.configure(
                elevatorConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Elevator Subsystem/Left Encoder Position", leftEncoder.getPosition());
        Logger.recordOutput("Elevator Subsystem/Right Encoder Position", rightEncoder.getPosition());
        Logger.recordOutput("Elevator Subsystem/Left Encoder Velocity", leftEncoder.getVelocity());
        Logger.recordOutput("Elevator Subsystem/Right Encoder Velocity", rightEncoder.getVelocity());
    }

    public Command setElevateCommand(int newPosition) {
        return this.runOnce(() -> {
            this.currentPosition = newPosition;
            updateElevator();
        });
    }

    public Command changeElevateCommand(int positionChange) {
        return this.runOnce(() -> {
            this.currentPosition = Math.min(this.currentPosition + positionChange, 2);
            updateElevator();
        });
    }

    public void updateElevator() {
        double position = elevatorPositions[this.currentPosition] * elevatorConversion;

        leftController.setReference(position, ControlType.kPosition);
        rightController.setReference(position, ControlType.kPosition);
    }
}
