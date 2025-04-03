package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.LightsSubsystem.LightsState;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  private static final double kP = 0.1;

  private final SparkMax climber = new SparkMax(RobotMap.CLIMBER, MotorType.kBrushless);

  private final SparkMaxConfig climbConfigs = new SparkMaxConfig();

  private final SparkClosedLoopController climbController = climber.getClosedLoopController();

  private final RelativeEncoder climbEncoder = climber.getEncoder();
  
  private ClimbGoal goal = ClimbGoal.STOW;
// Negative numbers go forward, Positive numbers go back
  public enum ClimbGoal {
    CLIMB(0.0),
    STOW(-370.0);

    private double goal;
    private ClimbGoal(double goal){
      this.goal = goal;
    }

    private double getGoal(){return goal;}
  }




 public ClimbSubsystem() {

  climbConfigs.closedLoop.p(kP).outputRange(-0.5, 0.5);

    climber.configure(
      climbConfigs.inverted(false),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    climbEncoder.setPosition(0);
  }

  public Command setClimbCommand(ClimbGoal goal) {
    return this.runOnce(() -> {
      this.goal = goal;
      updateClimb();
    });
  }

  public void updateClimb() {
    climbController.setReference(goal.getGoal(), ControlType.kPosition);
  }

  @Override
  public void periodic() {

    if(DriverStation.isDisabled()){
      climbConfigs.idleMode(SparkBaseConfig.IdleMode.kBrake);
      climber.configure(climbConfigs, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
        

    Logger.recordOutput("Climb Subsystem/Speed", climber.get());
    Logger.recordOutput("Climb Subsystem/Encoder Position", climbEncoder.getPosition());
    Logger.recordOutput("Climb Subsystem/Encoder Velocity", climbEncoder.getVelocity());
  }
}
