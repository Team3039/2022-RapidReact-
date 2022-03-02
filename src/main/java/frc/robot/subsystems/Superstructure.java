// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Turret.TurretState;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public static Drive mDrive = Drive.getInstance();
  public static Indexer mIndexer = Indexer.getInstance();
  public static Intake mIntake = Intake.getInstance();
  public static Shooter mShooter = Shooter.getInstance();
  public static Turret mTurret = Turret.getInstance();
  public static LEDs mLEDs = new LEDs();
  public static Superstructure INSTANCE = new Superstructure();

  public static SuperstructureState mState = SuperstructureState.IDLE;

  public static boolean isJammed = false;
  boolean hasOneBall;
  boolean hasTwoBalls;

  private final DigitalInput mFeederGate;
  private final DigitalInput mGripperGate;

  public Superstructure() {
    mFeederGate = new DigitalInput(Constants.RobotMap.hopperFeederGate);
    mGripperGate = new DigitalInput(Constants.RobotMap.hopperGripperGate);
  }

  public static enum SuperstructureState {
    IDLE,
    INTAKING,
    UNJAMMING,
    SHOOTING,
    CLIMBING
  }

  public static SuperstructureState getState() {
    return mState;
  }

  public void setState(SuperstructureState traverseState) {
    mState = traverseState;
  }

  public static Superstructure getInstance() {
    return INSTANCE;
  }

  public void indexIntake() {
    switch (Indexer.getInstance().getState()) {
      case ACTIVE_INDEXING:
        if (Intake.getInstance().isWrongBall())
          Intake.getInstance().setState(IntakeState.OUTTAKING);
        else {
          if (!hasTwoBalls)
            Intake.getInstance().setState(IntakeState.INTAKING);
          else
            Intake.getInstance().setState(IntakeState.IDLE);
        }
        break;
      case PASSIVE_INDEXING:
        Intake.getInstance().setState(IntakeState.INTAKING);
        break;
      case UNJAMMING:
        Intake.getInstance().setState(IntakeState.OUTTAKING);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    hasOneBall = !mFeederGate.get();
    hasTwoBalls = hasOneBall && !mGripperGate.get();

    switch (Superstructure.getInstance().getState()) {
      case CLIMBING:
        break;
      case IDLE:
        Indexer.getInstance().setState(IndexerState.IDLE);
        Intake.getInstance().setState(IntakeState.IDLE);
        Shooter.getInstance().setState(ShooterState.IDLE);
        Turret.getInstance().setState(TurretState.DRIVE);
        break;
      case INTAKING:
        Indexer.getInstance().setState(IndexerState.ACTIVE_INDEXING);
        indexIntake();
        break;
      case SHOOTING:
        Shooter.getInstance().setState(ShooterState.SHOOTING);
        Indexer.getInstance().setState(IndexerState.PASSIVE_INDEXING);
        Turret.getInstance().setState(TurretState.TRACKING);
        indexIntake();
        break;
      case UNJAMMING:
        Indexer.getInstance().setState(IndexerState.UNJAMMING);
        Shooter.getInstance().setState(ShooterState.UNJAMMING);
        indexIntake();
        break;
      default:
        break;
    }
  }
}
