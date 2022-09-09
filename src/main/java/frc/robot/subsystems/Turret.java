// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public MedianFilter mFilter = new MedianFilter(5);

  public static boolean isAtTargetPosition;

  /** Creates a new Turret. */
  public Turret() {
    turret.setSelectedSensorPosition(0);
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    turret.setNeutralMode(NeutralMode.Coast);

    turret.configForwardSoftLimitThreshold(degreesToTicks(90));
    turret.configReverseSoftLimitThreshold(degreesToTicks(-90));

    turret.configForwardSoftLimitEnable(true);
    turret.configReverseSoftLimitEnable(true);
    turret.setInverted(false);

    // Tracking
    turret.config_kP(0, .7);
    turret.config_kI(0, 0.0000001); 
    turret.config_kD(0, 12);
    
    // Off
    turret.config_kP(1, 0);
    turret.config_kI(1, 0);
    turret.config_kD(1, 0);

    // Zeroing
    turret.config_kP(2, 0.15);
    turret.config_kI(2, 0);
    turret.config_kD(2, 10);
  }

  public void setState(TurretState state) {
    this.turretState = state;
  }

  public TurretState getState() {
    return turretState;
  }

  public void trackTarget() {
    // turret.set(ControlMode.PercentOutput, -1 * targetX * Constants.Turret.kP_TURRET_TRACK);
    setTurretPosition(getCurrentAngle() - targetX);
  }

  // Assuming 0 is facing the opposite of the intake .
  // Start match turret facing backwards; 180 is facing the intake.

  // rotate turret to specified angle
  public void setTurretPosition(double targetAngle) {
    turret.set(ControlMode.Position, degreesToTicks(targetAngle));
  }

  public void stop() {
    turret.set(ControlMode.PercentOutput, 0);
  }

  public double degreesToTicks(double theta) {
    return theta * (22320 / 360.0);
  }

  public double getCurrentAngle() {
    return (turret.getSelectedSensorPosition() / 22320) * 360;
  }

  //  Experimental center field tracking. Gyro must be centered with intake facing away from you.
  // public void trackCenterField() {
  //   // double x = RobotContainer.drive.swerveOdometry.getPoseMeters().getX();
  //   // double y = RobotContainer.drive.swerveOdometry.getPoseMeters().getY();    

  //   double driveAngleToTarget = FieldOrientedTurretHelper.getAngleToTarget(RobotContainer.drive.swerveOdometry.getPoseMeters()).getDegrees();
  //   SmartDashboard.putNumber("Drive Error To Target", driveAngleToTarget);

  //   double targetAngle = driveAngleToTarget - RobotContainer.drive.swerveOdometry.getPoseMeters().getRotation().getDegrees();
  //   SmartDashboard.putNumber("Non Limelight Target Angle", targetAngle);

  //   turret.set(ControlMode.Position, targetAngle);
  // }

  public void resetEncoder() {
    turret.setSelectedSensorPosition(0);
  }

  public void resetSoftLimits(int fwd, int rev) {
    turret.configForwardSoftLimitThreshold(degreesToTicks(fwd));
    turret.configReverseSoftLimitThreshold(degreesToTicks(rev));
  }

  public void toggleSoftLimits(boolean isOn) {
    turret.configForwardSoftLimitEnable(isOn);
    turret.configReverseSoftLimitEnable(isOn);
  }

  // 0 tracking, 1 driver
  public void setCamMode(int val) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(val);
  }

  // 1 off, 3 on
  public void setLedMode(int val) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(val);
  }

  public void setPipeline(int val) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("Pipeline").setNumber(val);
  }

  @Override
  public void periodic() {
 
    SmartDashboard.putNumber("Current Angle", getCurrentAngle());
    SmartDashboard.putNumber("TargetX", targetX);
    SmartDashboard.putNumber("Turret Encoder", turret.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Voltage", turret.getMotorOutputVoltage());
    SmartDashboard.putNumber("Target Y", targetY);

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
        setCamMode(0);
        setLedMode(3);
        trackTarget();
        isAtTargetPosition = MathUtils.epsilonEquals(targetX, 0, 1.5);
        break;
      case DRIVE:
        // turret.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        turret.selectProfileSlot(2, 0);
        setCamMode(1);
        setLedMode(1);
        setTurretPosition(0);
        break;
      case MANUAL:
        turret.selectProfileSlot(1, 0);
        turret.set(ControlMode.PercentOutput, RobotContainer.getOperator().getRightXAxis() * -1);
        break;
      case CLIMBING:
        turret.selectProfileSlot(0, 0);
        // turret.setStatusFramePeriod(StatusFrame.Status_1_General, 253);
        // setTurretPosition(-180);
        break;
      default:
        setCamMode(1);
        setLedMode(1);
        stop();
    }
  }
}