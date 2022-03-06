// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;

public class SetUnjamming extends CommandBase {
  /** Creates a new SetUnjamming. */
  public SetUnjamming() {
    addRequirements(RobotContainer.mIndexer);
    addRequirements(RobotContainer.mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.mIndexer.setState(IndexerState.UNJAMMING);
    RobotContainer.mIntake.setState(IntakeState.OUTTAKING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mIndexer.setState(IndexerState.IDLE);
    RobotContainer.mIntake.setState(IntakeState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
