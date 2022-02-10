// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  TalonSRX intakeSpin = new TalonSRX(Constants.RobotMap.intake);
  Solenoid intakeTilt = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RobotMap.intakeTilt);

  /** Creates a new Intake. */
  public Intake() {
    intakeSpin.setNeutralMode(NeutralMode.Coast);
  }

  public void setIntakeSpeed(double speed) {
    intakeSpin.set(ControlMode.PercentOutput, speed);
  }

  public void actuateIntake(boolean Actuated) {
    intakeTilt.set(Actuated);
  }

  public void runIntake(double speed, boolean Actuated) {
    setIntakeSpeed(speed);
    actuateIntake(Actuated);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
