// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.MathUtils;
import frc.lib.util.Vector2;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

  // public static Shooter INSTANCE = new Shooter();
  public static ShooterState state = ShooterState.IDLE;

  public TalonFX leader = new TalonFX(Constants.Ports.SHOOTER_MASTER, "Drivetrain");
  public TalonFX follower = new TalonFX(Constants.Ports.SHOOTER_SLAVE, "Drivetrain");

  public Servo leftHood = new Servo(Constants.Ports.LEFT_HOOD);
  // public Servo rightHood = new Servo(Constants.Ports.RIGHT_HOOD);

  public static double mSetPoint = 4125;
  public static boolean isAtSetPoint;

  public static InterpolatingTreeMap<InterpolatingDouble, Vector2> shooterMap;

  /** Creates a new Shotoer. */
  public Shooter() {
    leader.setInverted(true);
    leader.setNeutralMode(NeutralMode.Coast);

    follower.setInverted(false);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.config_kP(0, 0.39);
    leader.config_kI(0, 0);
    leader.config_kD(0, 7.5);

    follower.follow(leader);

    shooterMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();
    shooterMap.put(new InterpolatingDouble(Double.valueOf(1000)), new Vector2(2, 2));

    leader.setStatusFramePeriod(StatusFrame.Status_1_General, 67);
    follower.setStatusFramePeriod(StatusFrame.Status_1_General, 79);
  }

  // public static Shooter getInstance() {
  //   return INSTANCE;
  // }

  public enum ShooterState {
    IDLE,
    SPIN_UP,
    UNJAMMING,
    SHOOTING,
    CLIMBING
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

  // Alter the setpoint based on observed deviations.
  public double offsetRPM(double setpoint) {
    return setpoint - (setpoint - 3000) / 4;
  }

  public void setHoodAngle(double pos) {
    leftHood.setPosition(pos);
    // rightHood.setAngle(angle);
  }

  @Override
  public void periodic() {

    // setHoodAngle(0);
  

 //   SmartDashboard.putNumber("Shooter SetPoint Velocity", RPMToVelocity(4500));
    // SmartDashboard.putNumber("Shooter Encoder", leader.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter Percent Speed", leader.getMotorOutputPercent());
    SmartDashboard.putNumber("Shooter RPM", velocityToRPM(leader.getSelectedSensorVelocity()));

    // SmartDashboard.putString("Shooter State", String.valueOf(getState()));

    switch (getState()) {
      case IDLE:
        leader.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        leader.set(ControlMode.PercentOutput, 0);
        break;
      case SHOOTING:
        leader.setStatusFramePeriod(StatusFrame.Status_1_General, 67);
        mSetPoint = shooterMap.getInterpolated(new InterpolatingDouble(Turret.targetY)).y; 
        leader.set(ControlMode.Velocity,
            RPMToVelocity(shooterMap.getInterpolated(new InterpolatingDouble(Turret.targetY)).y));
            
        isAtSetPoint = MathUtils.epsilonEquals(velocityToRPM(leader.getSelectedSensorVelocity()), offsetRPM(mSetPoint), 500);
        SmartDashboard.putBoolean("Is At Shooter Setpoint", isAtSetPoint);

        setHoodAngle(shooterMap.getInterpolated(new InterpolatingDouble(Turret.targetY)).x);
        break;
      case SPIN_UP:
        leader.setStatusFramePeriod(StatusFrame.Status_1_General, 67);
        leader.set(ControlMode.Velocity, RPMToVelocity(mSetPoint));
        isAtSetPoint = MathUtils.epsilonEquals(velocityToRPM(leader.getSelectedSensorVelocity()), offsetRPM(mSetPoint), 500);
        break;
      case UNJAMMING:
        leader.set(ControlMode.PercentOutput, -0.45);
        break;
      case CLIMBING:
      leader.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        leader.set(ControlMode.Disabled, 0);
        break;
      default:
        break;
    }
  }
}