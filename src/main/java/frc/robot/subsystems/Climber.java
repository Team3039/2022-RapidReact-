// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public TalonFX leftClimber = new TalonFX(Constants.Ports.CLIMB_MASTER);
    public TalonFX rightClimber = new TalonFX(Constants.Ports.CLIMB_SLAVE);

    public boolean areSoftLimitsEnabled = true;
    /** Creates a new Climber. */
    // Green goes down
    public Climber() {
        rightClimber.setInverted(true);

        leftClimber.setNeutralMode(NeutralMode.Brake);
        rightClimber.setNeutralMode(NeutralMode.Brake);

        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);

        // leftClimber.configReverseSoftLimitThreshold(0);
        leftClimber.configReverseSoftLimitEnable(false);

        // rightClimber.configReverseSoftLimitThreshold(0);
        rightClimber.configReverseSoftLimitEnable(false);

        // leader.configForwardSoft%LimitThreshold(Constants.Climber.TELESCOPING_ENCODER_LIMIT);
        // leader.configReverseSoftLimitThreshold(0);

        leftClimber.configForwardSoftLimitEnable(false);
        // leftClimber.configReverseSoftLimitEnable(true);

        // follower.configForwardSoftLimitThreshold(Constants.Climber.TELESCOPING_ENCODER_LIMIT);
        // follower.configReverseSoftLimitThreshold(0);

        rightClimber.configForwardSoftLimitEnable(false);
        // rightClimber.configReverseSoftLimitEnable(true);

        leftClimber.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        rightClimber.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
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
        // if (leftClimber.getSelectedSensorPosition() < 0 && percent <= 0 && areSoftLimitsEnabled) {
        //     leftClimber.set(ControlMode.PercentOutput, 0);
        // }
        // else {
        leftClimber.set(ControlMode.PercentOutput, percent);
        // }
    }

    public void setRightOutput(double percent) {
        // if (rightClimber.getSelectedSensorPosition() < 0 && percent <= 0 && areSoftLimitsEnabled) {
        //     rightClimber.set(ControlMode.PercentOutput, 0);
        // }
        // else {
        rightClimber.set(ControlMode.PercentOutput, percent);
        // }
    }

    public void setClimberPosition(double encoderPos) {
        leftClimber.set(ControlMode.Position, encoderPos);
        rightClimber.set(ControlMode.Position, encoderPos);
    }
    
    @Override
    public void periodic() {
        // if (RobotContainer.indexer.getState().equals(IndexerState.CLIMBING)) {
        //     RobotContainer.compressor.disable();
        //     setClimberOutput((RobotContainer.getOperator().getLeftYAxis()));
        // }

        SmartDashboard.putNumber("Climb encoder right", rightClimber.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb encoder left", leftClimber.getSelectedSensorPosition());
    }
}
       
