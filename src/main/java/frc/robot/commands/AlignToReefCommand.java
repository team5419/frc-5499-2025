/***********
 * https://www.chiefdelphi.com/t/from-zero-to-auto-align-to-reef-tutorial-how-to-simply-auto-align-to-the-reef/494478
 * https://github.com/ElectroBunny/BetaBot2025/blob/develop/src/main/java/frc/robot/commands/AlignToReefTagRelative.java
 ***********/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignToReefCommand extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveDriveSubsystem drivetrain;
  private double tagID = -1;

  // Robot relative drive request
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToReefCommand(boolean isRightScore, SwerveDriveSubsystem drivetrain) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", positions[2]);

      double xSpeed = xController.calculate(positions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      Translation2d translation = new Translation2d(xSpeed, ySpeed);

      drivetrain.applyRequest(() ->
        drive.withVelocityX(translation.getX())
          .withVelocityY(translation.getY())
          .withRotationalRate(rotValue)
      );

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      Translation2d translation = new Translation2d(0, 0);

      drivetrain.applyRequest(() ->
        drive.withVelocityX(translation.getX())
          .withVelocityY(translation.getY())
          .withRotationalRate(0)
      );
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    Translation2d translation = new Translation2d(0, 0);

    drivetrain.applyRequest(() ->
      drive.withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(0)
    );
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
