// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Indexer.IndexerState;

// public class Climber extends SubsystemBase {

//     public TalonSRX leader = new TalonSRX(Constants.Ports.CLIMB_MASTER);
//     public TalonSRX follower = new TalonSRX(Constants.Ports.CLIMB_SLAVE);

//     public Solenoid mActuatorA = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.CLIMB_ACTUATOR_A);
//     public Solenoid mActuatorB = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.CLIMB_ACTUATOR_B);

//     /** Creates a new Climber. */
//     public Climber() {
//         leader.setNeutralMode(NeutralMode.Brake);
//         follower.setNeutralMode(NeutralMode.Brake);

//         follower.follow(leader);

//         leader.configForwardSoftLimitThreshold(Constants.Climber.TELESCOPING_ENCODER_LIMIT);
//         leader.configReverseSoftLimitThreshold(0);

//         leader.configForwardSoftLimitEnable(true);
//         leader.configReverseSoftLimitEnable(true);
//     }

//     public void setClimberOutput(double percent) {
//         leader.set(ControlMode.PercentOutput, percent);
//     }

//     public void actuateClimb(boolean isActuated) {
//         mActuatorA.set(isActuated);
//         mActuatorB.set(isActuated);
//     }

//     @Override
//     public void periodic() {
//         if (RobotContainer.indexer.getState().equals(IndexerState.CLIMBING)) {
//             setClimberOutput(RobotContainer.getOperator().getLeftYAxis());
//         }
//     }
// }
