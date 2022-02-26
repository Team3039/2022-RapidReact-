// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

  TalonSRX frontBelt = new TalonSRX(Constants.RobotMap.hopperFrontBelt);
  TalonSRX backBelt = new TalonSRX(Constants.RobotMap.hopperBackBelt);
  TalonSRX frontWheel = new TalonSRX(Constants.RobotMap.hopperFrontWheel);

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  public DigitalInput topBeam = new DigitalInput(Constants.RobotMap.topBeam);
  public DigitalInput bottomBeam = new DigitalInput(Constants.RobotMap.bottomBeam);

  /** Creates a new Hopper. */
  public Hopper() {
    frontBelt.setNeutralMode(NeutralMode.Coast);
    backBelt.setNeutralMode(NeutralMode.Coast);
    frontWheel.setNeutralMode(NeutralMode.Coast);
  }

  /* Control Modes */
  public enum HopperControlMode {
    INDEXING,
    UNJAMMING,
    SHOOTING,
    IDLE,
    REJECTING
  }

  public HopperControlMode hopperControlMode = HopperControlMode.IDLE;

  public void setHopperControlMode(HopperControlMode controlMode) {
    hopperControlMode = controlMode;
  }

  public HopperControlMode getHopperControlMode() {
    return hopperControlMode;
  }

  /* Functional Methods */
  public void setBackBelt(double speed) {
    backBelt.set(ControlMode.PercentOutput, speed);
  }

  public void setFrontBelt(double speed) {
    frontBelt.set(ControlMode.PercentOutput, speed);
  }

  public void setFrontWheel(double speed) {
    frontWheel.set(ControlMode.PercentOutput, speed);
  }

  public void runFeeder(double FWSpeed, double FBSpeed, double BBSpeed) {
    frontWheel.set(ControlMode.PercentOutput, FWSpeed);
    frontBelt.set(ControlMode.PercentOutput, FBSpeed);
    backBelt.set(ControlMode.PercentOutput, BBSpeed);
  }

  public void setHopperSpeeds() {
    // 0.5 is a placeholder
    // both beams arent broken, so feed
    if (topBeam.get() && bottomBeam.get()) {
      runFeeder(0.5, 0.5, 0.5);
    }
    // bottom beam detects a ball but top doesnt, so feed to get that ball to the
    // top beam.
    if (topBeam.get() && !bottomBeam.get()) {
      runFeeder(0.5, 0.5, 0.5);
    }
    // top beam detects a ball but bottom doesnt. Only run the wheel to get the
    // second ball into position.
    if (!topBeam.get() && bottomBeam.get()) {
      setFrontWheel(0.5);
      setFrontBelt(0);
      setBackBelt(0);
    }
    // both beams are broken, so dont feed
    if (!topBeam.get() && !bottomBeam.get()) {
      runFeeder(0, 0, 0);
    }
  }

  public boolean getWrongBallDetection() {
    Color detectedColor = colorSensor.getColor();

    if(DriverStation.getAlliance().equals(Alliance.Blue)) {
      if(detectedColor.red > .4) {
        return true;
      } else {
        return false;
      }
    }
    if(DriverStation.getAlliance().equals(Alliance.Red)) {
      if(detectedColor.blue > .4) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  }


  @Override
  public void periodic() {

    switch (getHopperControlMode()) {
      case IDLE:
        runFeeder(0, 0, 0);
        break;
      case INDEXING:
        setHopperSpeeds();
        break;
      case SHOOTING:
        runFeeder(0.5, 0.5, 0.5);
        break;
      case UNJAMMING:
        runFeeder(-0.5, -0.5, -0.5);
        break;
      case REJECTING:
        runFeeder(-.75, 0, 0);
    }

  }
}
