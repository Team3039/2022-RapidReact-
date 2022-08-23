// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ToggleClimbSoftLimits;

public class Climber extends SubsystemBase {

    public TalonFX leftClimber = new TalonFX(Constants.Ports.LEFT_CLIMBER, "Drivetrain");
    public TalonFX rightClimber = new TalonFX(Constants.Ports.RIGHT_CLIMBER, "Drivetrain");

    public Solenoid stationarySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.CLIMB_STATIONARY_ACTUATOR);
    public Solenoid extendingSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.CLIMB_EXTENDING_ACTUATOR);
    

    public Climber() {
        leftClimber.setInverted(false);
        rightClimber.setInverted(true);

        leftClimber.setNeutralMode(NeutralMode.Brake);
        rightClimber.setNeutralMode(NeutralMode.Brake);

        setClimbEncoders(0); 

        leftClimber.config_kP(0, 0.8);
        leftClimber.config_kI(0, 0);
        leftClimber.config_kD(0, 0);

        rightClimber.config_kP(0, 0.8);
        rightClimber.config_kI(0, 0);
        rightClimber.config_kD(0, 0);

        toggleSoftLimits(false);
        leftClimber.configForwardSoftLimitThreshold(Constants.Climber.CLIMB_ENCODER_LIMIT);
        leftClimber.configReverseSoftLimitThreshold(0);
        rightClimber.configForwardSoftLimitThreshold(Constants.Climber.CLIMB_ENCODER_LIMIT);
        rightClimber.configReverseSoftLimitThreshold(0);
    }

    // the flipping arms
    public void releaseArms(boolean isRetracted) {
        stationarySolenoid.set(isRetracted);
    }

    // the extending arms
    public void actuateClimbers(boolean isRetracted) {
        extendingSolenoid.set(isRetracted);
    }

    public static double encoderToRotations(double value) {
        return value / Constants.Climber.ENCODER_TO_ROTATIONS_RATIO;
    }

    public static double rotationsToEncoder(double value) {
        return value * Constants.Climber.ENCODER_TO_ROTATIONS_RATIO;
    }

    public void setLeftOutput(double percent) {
        leftClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setRightOutput(double percent) {
        rightClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setLeftClimberPosition(double encoderPos) {
        leftClimber.set(ControlMode.Position, encoderPos);
    }

    public void setRightClimberPosition(double encoderPos) {
        rightClimber.set(ControlMode.Position, encoderPos);
    }

    public void setClimbEncoders(double value) {
        leftClimber.setSelectedSensorPosition(value);
        rightClimber.setSelectedSensorPosition(value);
    }

    // ticks
    public double getLeftClimberPosition() {
        return rotationsToEncoder(leftClimber.getSelectedSensorPosition());
    }

    // ticks
    public double getRightClimberPosition() {
        return rotationsToEncoder(rightClimber.getSelectedSensorPosition());
    }

    public void toggleSoftLimits(boolean isOn) {
        leftClimber.configForwardSoftLimitEnable(isOn);
        leftClimber.configReverseSoftLimitEnable(isOn);     
        rightClimber.configForwardSoftLimitEnable(isOn);
        rightClimber.configReverseSoftLimitEnable(isOn);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb encoder right", getRightClimberPosition());
        SmartDashboard.putNumber("Climb encoder left", getLeftClimberPosition());
    }
}
       
