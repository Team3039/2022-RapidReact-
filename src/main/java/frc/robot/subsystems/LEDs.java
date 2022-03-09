// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalOutput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;

// public class LEDs extends SubsystemBase {
//     /** Creates a new LEDs. */
//     DigitalOutput[] mOutputs = {
//             new DigitalOutput(1),
//             new DigitalOutput(2)
//     };

//     public boolean[] isIdle = {false, false};
//     public boolean[] isIntaking = {true, false};
//     public boolean[] isTargetFound = {false, true};
//     public boolean[] isClimbInitiated = {true, true};

//     public boolean[] mStates;

//     public LEDs() {
//     }

//     @Override
//     public void periodic() {
//         // switch (RobotContainer.mIndexer.getState()) {
//         //     case CLIMBING:
//         //         mStates = isClimbInitiated;
//         //         break;
//         //     case IDLE:
//         //         mStates = isIdle;
//         //         break;
//         //     case INDEXING:
//         //         mStates = isIntaking;
//         //         break;
//         //     case SHOOTING:
//         //         mStates = isTargetFound;
//         //         break;
//         //     case UNJAMMING:
//         //         mStates = isIntaking;
//         //         break;
//         //     default:
//         //         mStates = isIdle;
//         //         break;
//         // }
//         // for (int i = 0; i < 2; i++) {
//         //     mOutputs[i].set(mStates[i]);
//         // }
//     }
// }