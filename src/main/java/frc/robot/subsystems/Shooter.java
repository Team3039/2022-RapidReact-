// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public TalonFX shooter = new TalonFX(0);

  /** Creates a new Shotoer. */
  public Shooter() {
    shooter.setInverted(true);
    shooter.setNeutralMode(NeutralMode.Coast);

    shooter.config_kP(0, 0.5);
    shooter.config_kI(0, 0);
    shooter.config_kD(0, 0);
  }

  public BangBangController mWPIController = new BangBangController(0.1);

  public void setShooterSpeedBang(double setPoint) {
    shooter.set(ControlMode.PercentOutput,
        MathUtil.clamp(mWPIController.calculate(shooter.getSelectedSensorVelocity(), setPoint), 0, 0.8));
  }

  public void setShooterSpeed(double desiredVoltage) {
    shooter.set(ControlMode.PercentOutput, desiredVoltage);
  }

  public void setShooterSpeedPID() {
    shooter.getClosedLoopError();
  }

  @Override
  public void periodic() {

  }
}