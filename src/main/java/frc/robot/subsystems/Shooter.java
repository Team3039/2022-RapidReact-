// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.MathUtils;
import frc.lib.util.Vector2;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.IndexerState;

public class Shooter extends SubsystemBase {

  // public static Shooter INSTANCE = new Shooter();
  public static ShooterState state = ShooterState.IDLE;

  public TalonFX leader = new TalonFX(Constants.Ports.SHOOTER_LEADER, "Drivetrain");
  public TalonFX follower = new TalonFX(Constants.Ports.SHOOTER_FOLLOWER, "Drivetrain");

  public TalonSRX hood = new TalonSRX(Constants.Ports.HOOD);
  public CANCoder cancoder = new CANCoder(Constants.Ports.HOOD_CANCODER, "Drivetrain");

  public static double mSetPoint = 0;
  public static boolean isAtSetPoint;

  public static double mAngle;

  public static InterpolatingTreeMap<InterpolatingDouble, Vector2> shooterMap;

  private double[] mRPMRegressionVariable = {0};
  private double[] mHoodRegressionVariable = {0};


  /** Creates a new Shotoer. */
  public Shooter() {
    cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    cancoder.configSensorDirection(false);

    leader.setInverted(true);
    leader.setNeutralMode(NeutralMode.Coast);

    follower.setInverted(false);
    follower.setNeutralMode(NeutralMode.Coast);

    // With Flywheels
    leader.config_kP(0, 0.9);
    leader.config_kI(0, 0.00012);
    leader.config_kD(0, 6);

    // Without Flywheels
    // leader.config_kP(0, 0.4);
    // leader.config_kI(0, 0.0001);
    // leader.config_kD(0, 8);

    // Hood
    hood.config_kP(0, 0);
    hood.config_kI(0, 0);
    hood.config_kD(0, 0);


    follower.follow(leader);

    shooterMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();

    shooterMap.put(new InterpolatingDouble(Double.valueOf(-8.3)), new Vector2(2200, 0));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-14)), new Vector2(2600, 0.2));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-17)), new Vector2(2800, 0.4));
    // shooterMap.put(new InterpolatingDouble(Double.valueOf(4s)), new Vector2(4000, 0.4));
    // shooterMap.put(new InterpolatingDouble(Double.valueOf(1)), new Vector2(2000, 0.2));
    // shooterMap.put(new InterpolatingDouble(Double.valueOf(4)), new Vector2(4000, 0.4));
    // shooterMap.put(new InterpolatingDouble(Double.valueOf(1)), new Vector2(2000, 0.2));
    // shooterMap.put(new InterpolatingDouble(Double.valueOf(4)), new Vector2(4000, 0.4));
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
    follower.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }

  public void setShooterPercent(double percent) {
    follower.set(ControlMode.PercentOutput, percent);
  }

  public void setHoodAngle(double deg) {
    hood.set(ControlMode.Position, deg);
  }

  // public double calculateShooterRPM() {
  //   double output = 0;
  //   for (int i = 1; i < mRPMRegressionVariable.length; i++) {
  //     output += Math.pow(mRPMRegressionVariable[1], i);
  //   }
  //   return output;
  // }

  // public double calculateHoodAngle() {
  //   double output = 0;
  //   for (int i = 1; i < mHoodRegressionVariable.length; i++) {
  //     output += Math.pow(mHoodRegressionVariable[1], i);
  //   }
  //   return output;
  // }

  @Override
  public void periodic() {
    // System.out.println(shooterMap.getInterpolated(new InterpolatingDouble(Double.valueOf(3))));

    SmartDashboard.putNumber("Shooter SetPoint Velocity", velocityToRPM(leader.getSelectedSensorVelocity()));
    // SmartDashboard.putNumber("Shooter Encoder", leader.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter Percent Speed", leader.getMotorOutputPercent());

    // SmartDashboard.putString("Shooter State", String.valueOf(getState()));

    SmartDashboard.putNumber("Hood Cancoder Degrees", cancoder.getPosition());

    switch (getState()) {
      case IDLE:
        leader.set(ControlMode.PercentOutput, 0);
        follower.set(ControlMode.PercentOutput, 0);
        break;
      case SHOOTING:
        if (!RobotContainer.indexer.getState().equals(IndexerState.SHOOTING)) {
          mSetPoint = shooterMap.getInterpolated(new InterpolatingDouble(Turret.targetY)).x;
          mAngle = shooterMap.getInterpolated(new InterpolatingDouble(Turret.targetY)).y;
        }

        follower.set(ControlMode.Velocity, RPMToVelocity(mSetPoint));
            
        isAtSetPoint = MathUtils.epsilonEquals(velocityToRPM(leader.getSelectedSensorVelocity()), mSetPoint, 100);
        SmartDashboard.putBoolean("Is At Shooter Setpoint", isAtSetPoint);

        setHoodAngle(mAngle);
        break;
      case SPIN_UP:
        leader.set(ControlMode.Velocity, RPMToVelocity(mSetPoint));
        isAtSetPoint = MathUtils.epsilonEquals(velocityToRPM(follower.getSelectedSensorVelocity()), mSetPoint, 100);
        break;
      case UNJAMMING:
        leader.set(ControlMode.PercentOutput, -0.45);
        break;
      case CLIMBING:
        leader.set(ControlMode.Disabled, 0);
        break;
      default:
        break;
    }
  }
}