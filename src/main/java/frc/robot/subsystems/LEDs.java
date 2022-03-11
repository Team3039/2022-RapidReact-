// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
    /** Creates a new LEDs. */
    DigitalOutput[] mOutputs = {
            new DigitalOutput(Constants.Ports.LED_OUTPUT_A),
            new DigitalOutput(Constants.Ports.LED_OUTPUT_B)
    };

    public boolean[] isIdle = {false, false};
    public boolean[] isIntaking = {true, false};
    public boolean[] isTargetFound = {false, true};
    public boolean[] isClimbInitiated = {true, true};

    public boolean[] states;

    public LEDs() {
    }

    @Override
    public void periodic() {
        switch (RobotContainer.indexer.getState()) {
            case CLIMBING:
                states = isClimbInitiated;
                break;
            case IDLE:
                states = isIdle;
                break;
            case INDEXING:
                states = isIntaking;
                break;
            case SHOOTING:
                states = isTargetFound;
                break;
            case UNJAMMING:
                states = isIntaking;
                break;
            default:
                states = isIdle;
                break;
        }
        for (int i = 0; i < 2; i++) {
            mOutputs[i].set(states[i]);
        }
    }
}