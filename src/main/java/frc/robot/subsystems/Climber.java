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

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public TalonSRX elevatorA = new TalonSRX(Constants.RobotMap.climberRaiseA);
  public TalonSRX elevatorB = new TalonSRX(Constants.RobotMap.climberRaiseB);

  public Solenoid tiltA = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RobotMap.climberTiltA);
  public Solenoid tiltB = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RobotMap.climberTiltB);

  public Climber() {
    elevatorA.setNeutralMode(NeutralMode.Brake);
    elevatorB.setNeutralMode(NeutralMode.Brake);

    elevatorB.follow(elevatorA);
    
  }
  
  public void setElevator(double voltage) {
    elevatorA.set(ControlMode.PercentOutput, voltage);
  }

  public void tiltElevator(boolean isTilted) {
    tiltA.set(isTilted);
    tiltB.set(isTilted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
