// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.OptionalDouble;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Rotation2;
import frc.lib.util.Limelight.CamMode;
import frc.lib.util.Limelight.LedMode;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  public enum TurretMode {
    TRACKING,
    DRIVE,
    LEFT,
    RIGHT
  }

<<<<<<< HEAD
  public static Turret INSTANCE = new Turret();
=======
  public double turretAngle;
  private TurretMode turretmode = TurretMode.DRIVE;
>>>>>>> parent of 4fcc90b (-Minor Edits)

  public static Turret getInstance() {
    return INSTANCE;
  }

  private TurretState turretState = TurretState.DRIVE;

  TalonSRX mTurret = new TalonSRX(Constants.RobotMap.turret);

  public Turret() {
    // mTurret.configForwardSoftLimitThreshold(1000);
    // mTurret.configForwardSoftLimitEnable(true);

    // mTurret.configReverseSoftLimitThreshold(1000);
    // mTurret.configReverseSoftLimitEnable(true);
  }

  public void setTurretMode(TurretMode Mode) {
    this.turretmode = Mode;
  }

  public TurretMode getTurretMode() {
    return turretmode;
  }

  public void trackTarget() {
    if (LLDriver.getInstance().getAngleToTarget() != null) {
      // mTurret.set(
          // ControlMode.Position,
          // degreesToTicks(LLDriver.getInstance().getAngleToTarget().getAsDouble()));
    }
  }

  public void setPose(double degrees) {
    // mTurret.set(ControlMode.Position, degreesToTicks(degrees));
  }

  public double degreesToTicks(double degrees) {
    return degrees * Constants.Turret.TURRET_TO_ENCODER_RATIO * (2048 / 360);
  }

  @Override
  public void periodic() {
<<<<<<< HEAD
    switch (getTurretState()) {
=======
    // This method will be called once per scheduler run
    targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("TurretEncoder", turret.getSelectedSensorPosition());

    TurretMode currentMode = getTurretMode();
    switch (currentMode) {
>>>>>>> parent of 4fcc90b (-Minor Edits)
      case TRACKING:
        LLDriver.getInstance().setLedMode(LedMode.ON);
        LLDriver.getInstance().setCamMode(CamMode.VISION);
        trackTarget();
        break;
      case DRIVE:
        LLDriver.getInstance().setLedMode(LedMode.OFF);
        LLDriver.getInstance().setCamMode(CamMode.DRIVER);
        setPose(0);
        break;
      default:
        break;
    }
  }
}
