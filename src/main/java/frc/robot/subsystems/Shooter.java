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
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.MathUtils;
import frc.lib.util.Vector2;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SetHoodAngle;
import frc.robot.subsystems.Indexer.IndexerState;

public class Shooter extends SubsystemBase {

  // public static Shooter INSTANCE = new Shooter();
  public static ShooterState state = ShooterState.IDLE;

  public TalonFX leader = new TalonFX(Constants.Ports.SHOOTER_LEADER, "Drivetrain");
  public TalonFX follower = new TalonFX(Constants.Ports.SHOOTER_FOLLOWER, "Drivetrain");

  public TalonSRX hood = new TalonSRX(Constants.Ports.HOOD);
  public CANCoder cancoder = new CANCoder(Constants.Ports.HOOD_CANCODER, "Drivetrain");

  public static double setPointShooter = 0;
  public static double setPointHood = 0;

  public static boolean isAtSetPoint;
  public static boolean controllerHasBeenReset = false;

  public static PIDController hoodController;

  public static InterpolatingTreeMap<InterpolatingDouble, Vector2> shooterMap;

  /** Creates a new Shotoer. */
  public Shooter() {
    cancoder.configFeedbackCoefficient(1, "4096 cpr", SensorTimeBase.Per100Ms_Legacy);
    cancoder.configMagnetOffset(Constants.Shooter.HOOD_CANCODER_OFFEST);
    cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    cancoder.configSensorDirection(true);

    cancoder.setPositionToAbsolute();
    // cancoder.setPosition(0);

    leader.setInverted(false);
    leader.setNeutralMode(NeutralMode.Coast);

    follower.setInverted(true);
    follower.setNeutralMode(NeutralMode.Coast);
 
    hood.setNeutralMode(NeutralMode.Brake);

    // With Flywheels
    leader.config_kP(0, 1);
    leader.config_kI(0, 0.000015);
    leader.config_kD(0, 15);
    leader.config_kF(0, 0.05669);

    follower.config_kP(0, 1);
    follower.config_kI(0, 0.000015);
    follower.config_kD(0, 15);
    follower.config_kF(0, 0.05669);

    // hoodController = new ProfiledPIDController(0.0005, 0.0, .000005, new TrapezoidProfile.Constraints(16000.0, 12000.0));
    hoodController = new PIDController(0.0005, 0.00008, .000015);

    follower.follow(leader);

    shooterMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();

    shooterMap.put(new InterpolatingDouble(Double.valueOf(-0.23)), new Vector2(2550, 200));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-4.43)), new Vector2(2800, 800));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-7.56)), new Vector2(2800, 1100));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-9.89)), new Vector2(2900, 1100));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-12)), new Vector2(2810, 1100));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-13.83)), new Vector2(2850, 1139));
    shooterMap.put(new InterpolatingDouble(Double.valueOf(-15.35)), new Vector2(2985, 1750));
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
    return velocity / Constants.Shooter.SHOOTER_TO_ENCODER_RATIO / Constants.Shooter.SHOOTER_TICKS_PER_ROTATION * 600;
  }

  public double RPMToVelocity(double rpm) {
    return rpm * Constants.Shooter.SHOOTER_TO_ENCODER_RATIO * Constants.Shooter.SHOOTER_TICKS_PER_ROTATION / 600;
  }

  public void setShooterRPM(double rpm) {
    follower.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }

  public void setShooterPercent(double percent) {
    follower.set(ControlMode.PercentOutput, percent);
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

  public void setHoodAngle(double target) {
    double output = hoodController.calculate(cancoder.getPosition(), target);
    if (cancoder.getPosition() < target) {
      output += 0.031;
    }
    if (cancoder.getPosition() > target) {
      output -= 0.031;
    }
    System.out.println(output);
    output = hoodSoftLimits(output);
    hood.set(ControlMode.PercentOutput, MathUtils.clamp(output, -0.3, 0.3));
  }

  public void manualHood() {
    double output = RobotContainer.getOperator().getLeftYAxis() * 0.25;
    // output = hoodSoftLimits(output);
    hood.set(ControlMode.PercentOutput, output);
  }

  public double hoodSoftLimits(double output) {
    if (cancoder.getPosition() >= Constants.Shooter.HOOD_FORWARD_LIMIT_DEG && output > 0) {
      hood.set(ControlMode.PercentOutput, 0);
      return 0;
    }
    else if (cancoder.getPosition() <= Constants.Shooter.HOOD_REVERSE_LIMIT_DEG && output < 0) {
      hood.set(ControlMode.PercentOutput, 0);
      return 0;
    }
    else {
      return output;
    }
  }

  @Override
  public void periodic() {
    if (!RobotContainer.indexer.getState().equals(IndexerState.SHOOTING)) {
      setPointShooter = shooterMap.getInterpolated(new InterpolatingDouble(Turret.targetY)).x;
      setPointHood = shooterMap.getInterpolated(new InterpolatingDouble(Turret.targetY)).y;
    }
    
    SmartDashboard.putNumber("Leader Shooter Output", leader.getMotorOutputVoltage());
    SmartDashboard.putNumber("Follower Shooter Output", follower.getMotorOutputVoltage());

    // System.out.println(shooterMap.getInterpolated(new InterpolatingDouble(Double.valueOf(3))));

    SmartDashboard.putNumber("Shooter Velocity", velocityToRPM(leader.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter Target Velocity", setPointShooter);
    SmartDashboard.putNumber("Shooter Encoder", leader.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter Percent Speed", leader.getMotorOutputPercent());

    // SmartDashboard.putString("Shooter State", String.valueOf(getState()));

    SmartDashboard.putNumber("Hood Relative Ticks", cancoder.getPosition());
    SmartDashboard.putNumber("Hood Absolute Ticks", cancoder.getAbsolutePosition());
    SmartDashboard.putNumber("Target Hood Pos", setPointHood);

    switch (getState()) {
      case IDLE:
        leader.set(ControlMode.PercentOutput, 0);
        follower.set(ControlMode.PercentOutput, 0);
        if (!controllerHasBeenReset) {
          hoodController.reset();
          controllerHasBeenReset = true;
        }
        setHoodAngle(0);
        break;
      case SHOOTING:
        controllerHasBeenReset = false;
        leader.set(ControlMode.Velocity, RPMToVelocity(2800));
        follower.set(ControlMode.Velocity, RPMToVelocity(2800));
            
        isAtSetPoint = MathUtils.epsilonEquals(velocityToRPM(leader.getSelectedSensorVelocity()), setPointShooter, 25) &&  
                       MathUtils.epsilonEquals(cancoder.getPosition(), setPointHood, 20);
        SmartDashboard.putBoolean("Is At Shooter Setpoint", isAtSetPoint);
        
        setHoodAngle(800);
        break;
      case SPIN_UP:
        leader.set(ControlMode.Velocity, RPMToVelocity(setPointShooter));
        follower.set(ControlMode.Velocity, RPMToVelocity(setPointShooter));
        isAtSetPoint = MathUtils.epsilonEquals(velocityToRPM(follower.getSelectedSensorVelocity()), setPointShooter, 100);
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