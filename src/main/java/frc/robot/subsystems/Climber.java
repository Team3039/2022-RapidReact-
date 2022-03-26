// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public TalonFX leftClimber = new TalonFX(Constants.Ports.CLIMB_MASTER, "Drivetrain");
    public TalonFX rightClimber = new TalonFX(Constants.Ports.CLIMB_SLAVE, "Drivetrain");

    public Solenoid actuator = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.CLIMB_ACTUATOR);

    /** Creates a new Climber. */
    // Green goes down
    public Climber() {
        rightClimber.setInverted(true);

        leftClimber.setNeutralMode(NeutralMode.Brake);
        rightClimber.setNeutralMode(NeutralMode.Brake);

        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);

        leftClimber.configReverseSoftLimitThreshold(0);
        leftClimber.configReverseSoftLimitEnable(true);

        rightClimber.configReverseSoftLimitThreshold(0);
        rightClimber.configReverseSoftLimitEnable(true);

        rightClimber.configForwardSoftLimitThreshold(Constants.Climber.CLIMB_ENCODER_LIMIT);
        rightClimber.configForwardSoftLimitEnable(true);
  
        leftClimber.configForwardSoftLimitThreshold(Constants.Climber.CLIMB_ENCODER_LIMIT);
        leftClimber.configForwardSoftLimitEnable(true);

        leftClimber.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        rightClimber.setStatusFramePeriod(StatusFrame.Status_1_General, 10);

        leftClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        rightClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        // leftClimber.config_kP(0, 0);
        // rightClimber.config_kP(0, 0);
    }

    // public void setClimberOutput(double percent) {
        // if ((leftClimber.getSelectedSensorPosition() < 0 || rightClimber.getSelectedSensorPosition() < 0) && percent <= 0 && areSoftLimitsEnabled) {
        //     leftClimber.set(ControlMode.PercentOutput, 0);
        //     rightClimber.set(ControlMode.PercentOutput, 0);
        // }
        // else {
        // leftClimber.set(ControlMode.PercentOutput, percent);
        // rightClimber.set(ControlMode.PercentOutput, percent);
        // }
    // }

    public void setLeftOutput(double percent) {
        leftClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setRightOutput(double percent) {
        rightClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setClimberPosition(double encoderPos) {
        leftClimber.set(ControlMode.Position, encoderPos);
        rightClimber.set(ControlMode.Position, encoderPos);
    }

    public void setClimbEncoders(double value) {
        leftClimber.setSelectedSensorPosition(value);
        rightClimber.setSelectedSensorPosition(value);
    }

    public void actuateClimb(boolean isActuated) {
        actuator.set(isActuated);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb encoder right", rightClimber.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb encoder left", leftClimber.getSelectedSensorPosition());
    }
}
       
