// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.Vector2;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Shooter extends SubsystemBase {

  public static Shooter INSTANCE = new Shooter();
  public static ShooterState mState = ShooterState.IDLE;

  public TalonFX mLeader = new TalonFX(Constants.Ports.SHOOTER_MASTER);
  public TalonFX mFollower = new TalonFX(Constants.Ports.SHOOTER_SLAVE);

  public static InterpolatingTreeMap<InterpolatingDouble, Vector2> mShooterMap;
  /** Creates a new Shotoer. */
  public Shooter() {
    mLeader.setInverted(false);
    mLeader.setNeutralMode(NeutralMode.Coast);

    mFollower.setInverted(true);
    mFollower.setNeutralMode(NeutralMode.Coast);

    mLeader.config_kP(0, 0.5);
    mLeader.config_kI(0, 0);
    mLeader.config_kD(0, 0);

    mFollower.follow(mLeader);

    mShooterMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();
    mShooterMap.put(new InterpolatingDouble(Double.valueOf(1000)), new Vector2(2, 2));
  }

  public static Shooter getInstance() {
    return INSTANCE;
  }

  public enum ShooterState {
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
    mLeader.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }

  public void setShooterPercent(double percent) {
    mLeader.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    switch (Shooter.getInstance().getState()) {
      case IDLE:
        mLeader.set(ControlMode.PercentOutput, 0);
        break;
      case SHOOTING:
        setShooterPercent(0.75);
       // mMaster.set(TalonFXControlMode.Velocity, RPMToVelocity(mShooterMap.getInterpolated(new InterpolatingDouble(RobotContainer.mLimelight.getDistanceToTarget().getAsDouble())).x));
        break;
      case SPIN_UP:
        break;
      case UNJAMMING:
        mLeader.set(ControlMode.PercentOutput, 0.45);
      default:
        break;
    }
  }
}