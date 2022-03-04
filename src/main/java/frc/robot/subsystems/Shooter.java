// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  public TalonFX shooter = new TalonFX(Constants.RobotMap.shooter);

  /** Creates a new Shotoer. */
  public Shooter() {
    shooter.setInverted(true);
    shooter.setNeutralMode(NeutralMode.Coast);

    shooter.config_kP(0, 0.5);
    shooter.config_kI(0, 0);
    shooter.config_kD(0, 0);
  }

  // Velocity in ticks per 100ms  (getSelectedSensorVelocity)
  public double velocityToRPM(double velocity) {
    return velocity / Constants.Shooter.SHOOTER_TO_ENCODER_RATIO / Constants.Shooter.TICKS_PER_ROTATION * 600;
  }
  
  public double RPMToVelocity(double rpm) {
    return rpm * Constants.Shooter.SHOOTER_TO_ENCODER_RATIO * Constants.Shooter.TICKS_PER_ROTATION / 600;
 }

  public void setShooterRPM(double rpm) {
    shooter.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }


  @Override
  public void periodic() {

  }
}