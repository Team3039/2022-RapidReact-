// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ToggleClimbSoftLimits extends CommandBase {
  /** Creates a new ToggleClimbSoftLimits. */
  public ToggleClimbSoftLimits() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);
    RobotContainer.climber.leftClimber.enableSoftLimit(SoftLimitDirection.kForward, false);
    RobotContainer.climber.rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);
    RobotContainer.climber.rightClimber.enableSoftLimit(SoftLimitDirection.kForward, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setClimbEncoders(0);
    RobotContainer.climber.leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);
    RobotContainer.climber.leftClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
    RobotContainer.climber.rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);
    RobotContainer.climber.rightClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
