// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
    /** Creates a new LEDs. */
    DigitalOutput[] outputs = {
            new DigitalOutput(Constants.Ports.LED_OUTPUT_A),
            new DigitalOutput(Constants.Ports.LED_OUTPUT_B)
    };
    // Active when the robot is disabled or when the shooter is spinning up
    public boolean[] isAwaiting = { true, true };
    // Active when the shooter is at the correct speed and the turret is in the right position
    public boolean[] isAtShooterSetpoint = { false, true };
    // Active when the robot is climbing
    public boolean[] isClimbInitiated = { true, false };
    // Active whenever the robot is enabled and does not meet the above cases
    public boolean[] isIdle = { false, false };

    public boolean[] states;

    public LEDs() {
    }

    @Override
    public void periodic() {
            switch (Shooter.getState()) {
                case CLIMBING:
                    states = isClimbInitiated;
                    break;
                case IDLE:
                    states = isIdle;
                    break;
                case SHOOTING:
                    if (Shooter.isAtSetPoint && Turret.isAtTargetPosition) {
                        states = isAtShooterSetpoint;
                    } else {
                        states = isIdle;
                    }
                    break;
                case SPIN_UP:
                    if (Shooter.isAtSetPoint && Turret.isAtTargetPosition) {
                        states = isAtShooterSetpoint;
                    } else {
                        states = isIdle;
                    }
                    break;
                case UNJAMMING:
                    states = isIdle;
                    break;
                }
        for (int i = 0; i < 2; i++) {
            outputs[i].set(states[i]);
        }
    }
}