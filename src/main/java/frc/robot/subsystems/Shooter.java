// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public static Shooter instance = new Shooter();
  public static ShooterState state = ShooterState.IDLE;

  public TalonFX leader = new TalonFX(Constants.RobotMap.shooterA);
  public TalonFX follower = new TalonFX(Constants.RobotMap.shooterB);

  public double setpoint = 0;
  public boolean atSetpoint = false;


  /** Creates a new Shotoer. */
  public Shooter() {
    leader.setInverted(false);
    leader.setNeutralMode(NeutralMode.Coast);

    follower.setInverted(true);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.config_kP(0, 0.5);
    leader.config_kI(0, 0);
    leader.config_kD(0, 0);

    follower.follow(leader);
  }

  public static Shooter getInstance() {
    return instance;
  }

  public static enum ShooterState {
    IDLE,
    SPIN_UP,
    UNJAMMING,
    SHOOTING
  }

  public static ShooterState getState() {
    return state;
  }

  public void setState(ShooterState traverseState) {
    state = traverseState;
  }

  public double velocityToRPM(double velocity) {
    return velocity / Constants.Shooter.SHOOTER_TO_ENCODER_RATIO / Constants.Shooter.TICKS_PER_ROTATION * 600;
  }

  public double RPMToVelocity(double rpm) {
    return rpm * Constants.Shooter.SHOOTER_TO_ENCODER_RATIO * Constants.Shooter.TICKS_PER_ROTATION / 600;
  }

  public void setShooterRPM(double rpm) {
    leader.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }

  public void setShooterPercent(double percent) {
    leader.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", velocityToRPM(leader.getSelectedSensorVelocity()));
    switch (getState()) {
      case IDLE:
        leader.set(ControlMode.PercentOutput, 0);
        break;
      case SHOOTING:
        setShooterPercent(.65);
        break;
      case SPIN_UP:
        leader.set(ControlMode.PercentOutput, 0.5);
        break;
      case UNJAMMING:
        leader.set(ControlMode.PercentOutput, -0.45);
      default:
        break;
    }
  }
}