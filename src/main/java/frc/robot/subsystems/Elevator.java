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
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  private final SparkMax leftElevator = new SparkMax(RobotMap.LEFT_ELEVATOR, MotorType.kBrushless);
  private final SparkMax rightElevator =
      new SparkMax(RobotMap.RIGHT_ELEVATOR, MotorType.kBrushless);

  private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController leftController = leftElevator.getClosedLoopController();
  private final SparkClosedLoopController rightController = rightElevator.getClosedLoopController();

  private final RelativeEncoder leftEncoder = leftElevator.getEncoder();
  private final RelativeEncoder rightEncoder = rightElevator.getEncoder();

  private int currentPosition = 0;

  public Elevator() {
    elevatorConfig.closedLoop.p(0.1).outputRange(-1, 1);

    rightElevator.configure(
        elevatorConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    leftElevator.configure(
        elevatorConfig.inverted(false),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set Smart Motion / Smart Velocity parameters
    // int smartMotionSlot = 0;
    // m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    // m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    // m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    // m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // leftController.setReference(0, ControlType.kMAXMotionPositionControl);
    // rightController.setReference(0, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // System.out.println(leftEncoder.getPosition());
  }

  public Command setElevateCommand(int newPosition) {
    return this.runOnce(
        () -> {
          this.currentPosition = newPosition;
          double position = Constants.elevatorPositions[this.currentPosition];

          leftController.setReference(position, ControlType.kPosition);
          rightController.setReference(position, ControlType.kPosition);
        });
  }

  public Command changeElevateCommand(int positionChange) {
    return this.runOnce(
        () -> {
          this.currentPosition = Math.min(this.currentPosition + positionChange, 2);
          double position = Constants.elevatorPositions[this.currentPosition];

          leftController.setReference(
              position, ControlType.kPosition /*, ClosedLoopSlot.kSlot0, 1.0 */);
          rightController.setReference(
              position, ControlType.kPosition /*, ClosedLoopSlot.kSlot0, 1.0 */);
          // leftController.setReference(position, ControlType.kPosition);
          // rightController.setReference(position, ControlType.kPosition);
        });
  }
}
