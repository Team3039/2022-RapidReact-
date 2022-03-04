// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.Vector2;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public static Shooter INSTANCE = new Shooter();
  public static ShooterState mState = ShooterState.IDLE;

  public TalonFX mMaster = new TalonFX(Constants.RobotMap.shooterA);
  public TalonFX mSlave = new TalonFX(Constants.RobotMap.shooterB);

  public double setpoint = 0;
  public boolean atSetpoint = false;

  public static InterpolatingTreeMap<InterpolatingDouble, Vector2> mShooterMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();

  /** Creates a new Shotoer. */
  public Shooter() {
    // mMaster.setInverted(true);
    // mMaster.setNeutralMode(NeutralMode.Coast);

    // mSlave.follow(mMaster);

    // mMaster.config_kP(0, 0.5);
    // mMaster.config_kI(0, 0);
    // mMaster.config_kD(0, 0);

    // mShooterMap.put(new InterpolatingDouble(Double.valueOf(1000)), new Vector2(2, 2));
  }

  public static Shooter getInstance() {
    return INSTANCE;
  }

  public static enum ShooterState {
    IDLE,
    SPIN_UP,
    UNJAMMING,
    SHOOTING
  }

  public static ShooterState getState() {
    return mState;
  }

  public void setState(ShooterState traverseState) {
    mState = traverseState;
  }

  public double velocityToRPM(double velocity) {
    return velocity / Constants.Shooter.SHOOTER_TO_ENCODER_RATIO / Constants.Shooter.TICKS_PER_ROTATION * 600;
  }

  public double RPMToVelocity(double rpm) {
    return rpm * Constants.Shooter.SHOOTER_TO_ENCODER_RATIO * Constants.Shooter.TICKS_PER_ROTATION / 600;
  }

  public void setShooterRPM(double rpm) {
    mMaster.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }

  public Vector2 calculateShooterOutput(double distance) {
    return mShooterMap.getInterpolated(new InterpolatingDouble(Double.valueOf(distance)));
  }

  @Override
  public void periodic() {
    switch (getState()) {
      case IDLE:
        // mMaster.set(ControlMode.PercentOutput, 0);
        break;
      case SHOOTING:
        // mMaster.set(ControlMode.Velocity, RPMToVelocity(
            // mShooterMap.getInterpolated(
                // new InterpolatingDouble(
                    // Double.valueOf(
                        // LLDriver.getInstance().getDistanceToTarget().getAsDouble()))).y));
        break;
      case SPIN_UP:
        // mMaster.set(ControlMode.PercentOutput, 0.5);
        break;
      case UNJAMMING:
        // mMaster.set(ControlMode.PercentOutput, 0.45);
      default:
        break;
    }
  }
}