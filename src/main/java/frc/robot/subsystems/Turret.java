// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.FieldOrientedTurretHelper;
import frc.lib.util.Limelight.CamMode;
import frc.lib.util.Limelight.LedMode;
import frc.lib.util.MathUtils;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {

  public enum TurretState {
    TRACKING,
    DRIVE,
    MANUAL,
    CLIMBING
  }

  public static double targetValid; // Whether the limelight has any valid targets (0 or 1)
  public static double targetX; // HorizontalDR Offset From Crosshair To Target (-27 degrees to 27 degrees
  public static double targetY; // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  public static double targetArea; // Target Area (0% of image to 100% of image)

  private TurretState turretState = TurretState.DRIVE;

  public TalonSRX turret = new TalonSRX(Constants.Ports.TURRET);

  public static boolean isAtTargetPosition;

  /** Creates a new Turret. */
  public Turret() {
    turret.setSelectedSensorPosition(0);
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    turret.setNeutralMode(NeutralMode.Coast);

    turret.configForwardSoftLimitThreshold(degreesToTicks(45));
    turret.configReverseSoftLimitThreshold(degreesToTicks(-45));

    turret.configForwardSoftLimitEnable(true);
    turret.configReverseSoftLimitEnable(true);

    turret.config_kP(0, 1);
    // turret.config_kD(0, 2);
    
    turret.config_kP(1, 0);
    turret.config_kD(1, 0);

    turret.selectProfileSlot(0, 0);

    turret.setStatusFramePeriod(StatusFrame.Status_1_General, 200);
  }

  public void setState(TurretState state) {
    this.turretState = state;
  }

  public TurretState getState() {
    return turretState;
  }

  public void trackTarget() {
    // double correctionX = -1 * targetX;
    // setTurretPosition(correctionX + ticksToDegrees());
    turret.set(ControlMode.PercentOutput, -1 * targetX * Constants.Turret.kP_TURRET_TRACK);
  }

  // Assuming 0 is facing the opposite of the intake .
  // Start match turret facing backwards; 180 is facing the intake.

  // rotate turret to specified angle
  public void setTurretPosition(double targetAngle) {
    if (targetAngle < -90 || targetAngle > 90) {
      System.out.println("Turret Stop");
      turret.set(ControlMode.PercentOutput, 0);
    } else {
      turret.set(ControlMode.Position, degreesToTicks(targetAngle));
    }
  }

  public void stop() {
    turret.set(ControlMode.PercentOutput, 0);
    RobotContainer.limelight.setCamMode(CamMode.DRIVER);
  }

  public double degreesToTicks(double theta) {
    return theta * (22320 / 360.0);
  }

  public double getCurrentAngle() {
    return (turret.getSelectedSensorPosition() / 22320) * 360;
  }

  // Experimental center field tracking. Gyro must be centered with intake facing away from you.
  public void trackCenterField() {
    // double x = RobotContainer.drive.swerveOdometry.getPoseMeters().getX();
    // double y = RobotContainer.drive.swerveOdometry.getPoseMeters().getY();

    double driveAngleToTarget = FieldOrientedTurretHelper.getAngleToTarget(RobotContainer.drive.swerveOdometry.getPoseMeters()).getDegrees();
    SmartDashboard.putNumber("Drive Error To Target", driveAngleToTarget);

    double targetAngle = driveAngleToTarget - RobotContainer.drive.swerveOdometry.getPoseMeters().getRotation().getDegrees();
    SmartDashboard.putNumber("Non Limelight Target Angle", targetAngle);

    turret.set(ControlMode.PercentOutput, targetAngle);
  }

  public void resetEncoder() {
    turret.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("TurretEncoder", turret.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Desired Value", degreesToTicks(90));

    SmartDashboard.putNumber("Current Angle", getCurrentAngle());
    // SmartDashboard.putNumber("TargetX",
    // RobotContainer.limelight.getAngleToTarget().getAsDouble());

    SmartDashboard.putString("Turret State", String.valueOf(getState()));
    SmartDashboard.putBoolean("Is At Target Position", isAtTargetPosition);

    targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    // turret.set(ControlMode.PercentOutput, targetX *
    // Constants.Turret.kP_TURRET_TRACK);

    TurretState currentMode = getState();

    switch (currentMode) {
      case TRACKING:
        turret.selectProfileSlot(0, 0);
        RobotContainer.limelight.setCamMode(CamMode.VISION);
        RobotContainer.limelight.setLedMode(LedMode.ON);
        trackTarget();
        isAtTargetPosition = MathUtils.epsilonEquals(targetX, 0, 1);
        break;
      case DRIVE:
        turret.selectProfileSlot(0, 0);
        RobotContainer.limelight.setCamMode(CamMode.DRIVER);
        RobotContainer.limelight.setLedMode(LedMode.OFF);
        setTurretPosition(0);
        break;
      case MANUAL:
        turret.selectProfileSlot(1, 0);
        turret.set(ControlMode.PercentOutput, RobotContainer.getOperator().getRightXAxis() * -1);
        break;
      case CLIMBING:
        turret.set(ControlMode.Disabled, 0);
        break;
      default:
        RobotContainer.limelight.setCamMode(CamMode.DRIVER);
        stop();
    }
  }
}