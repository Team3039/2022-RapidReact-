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
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.IndexerState;

public class Climber extends SubsystemBase {

    public TalonFX leftClimber = new TalonFX(Constants.Ports.CLIMB_MASTER);
    public TalonFX rightClimber = new TalonFX(Constants.Ports.CLIMB_SLAVE);

    public Solenoid mActuatorA = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.CLIMB_ACTUATOR_A);
    public Solenoid mActuatorB = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.CLIMB_ACTUATOR_B);

    // public boolean isClimbing = false;

    /** Creates a new Climber. */
    // Green goes down
    public Climber() {
        rightClimber.setInverted(true);

        // follower.follow(leader);

        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);

        // leader.configForwardSoftLimitThreshold(Constants.Climber.TELESCOPING_ENCODER_LIMIT);
        // leader.configReverseSoftLimitThreshold(0);

        // leftClimber.configForwardSoftLimitEnable(true);
        // leftClimber.configReverseSoftLimitEnable(true);

        // follower.configForwardSoftLimitThreshold(Constants.Climber.TELESCOPING_ENCODER_LIMIT);
        // follower.configReverseSoftLimitThreshold(0);

        // rightClimber.configForwardSoftLimitEnable(true);
        // rightClimber.configReverseSoftLimitEnable(true);
    }

    public void setClimberOutput(double percent) {
        leftClimber.set(ControlMode.PercentOutput, percent);
        rightClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setLeftOutput(double percent) {
        leftClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setRightOutput(double percent) {
        rightClimber.set(ControlMode.PercentOutput, percent);
    }

    public void actuateClimb(boolean isActuated) {
        mActuatorA.set(isActuated);
        mActuatorB.set(isActuated);
    }

    @Override
    public void periodic() {
        // if (isClimbing) {
            setClimberOutput((RobotContainer.getOperator().getLeftYAxis() * -1) / 2);
        // }

        SmartDashboard.putNumber("Climb encoder right", rightClimber.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb encoder left", leftClimber.getSelectedSensorPosition());
    }
}
       
