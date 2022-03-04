package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeState;

public class Indexer extends SubsystemBase {
    
    public enum IndexerState {
        IDLE, PASSIVE_INDEXING,  ACTIVE_INDEXING, CLIMBING, HELLA_ZOOMING, UNJAMMING,
    }
    
    private static Indexer INSTANCE = new Indexer();

    private final TalonFX mGripper;
    private final TalonFX mFeeder;

    private IndexerState mState = IndexerState.IDLE;


    public Indexer() {
        mFeeder = new TalonFX(Constants.RobotMap.hopperFeeder);
        mGripper = new TalonFX(Constants.RobotMap.hopperGripper);

        // mFeeder.changeMotionControlFramePeriod(255);
        // mFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);
        // mFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 125);

        // mFeeder.configVoltageCompSaturation(12.0);
        // mFeeder.enableVoltageCompensation(true);
    }

    public synchronized static Indexer getInstance() {
        return INSTANCE;
    }

    public synchronized IndexerState getState() {
        return mState;
    }

    public void setState(IndexerState wanted_state) {
        mState = wanted_state;
    }

    public synchronized void setOpenLoop(
      double gripperOutput,
      double feederOutput) {
    //    mGripper.set(ControlMode.PercentOutput, gripperOutput);
    //    mFeeder.set(ControlMode.PercentOutput, feederOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
        // mFeeder.setNeutralMode(NeutralMode.Brake);
    }

    // public void indexIntake() {
    //     switch (Indexer.getInstance().getState()) {
    //       case ACTIVE_INDEXING:
    //         if (Intake.getInstance().isWrongBall())
    //           Intake.getInstance().setState(IntakeState.OUTTAKING);
    //         else {
    //           if (!Superstructure.getInstance().hasTwoBalls)
    //             Intake.getInstance().setState(IntakeState.INTAKING);
    //           else
    //             Intake.getInstance().setState(IntakeState.IDLE);
    //         }
    //         break;
    //       case PASSIVE_INDEXING:
    //         Intake.getInstance().setState(IntakeState.INTAKING);
    //         break;
    //       case UNJAMMING:
    //         Intake.getInstance().setState(IntakeState.OUTTAKING);
    //         break;
    //       default:
    //         break;
    //     }
    //   }

    @Override
    public void periodic() {
    //     switch (mState) {
    //         case IDLE:
    //             setOpenLoop(0, 0);
    //             break;
    //         case PASSIVE_INDEXING:
    //             setOpenLoop(0.5, 0.5);
    //             // indexIntake();
    //             break;
    //         case ACTIVE_INDEXING:
    //             if (!Superstructure.getInstance().hasOneBall && !Superstructure.getInstance().hasTwoBalls)
    //                 setOpenLoop(0.75, 0.75);
    //             else if (Superstructure.getInstance().hasOneBall && !Superstructure.getInstance().hasTwoBalls)
    //                 setOpenLoop(0.25, 0);
    //             else if (Superstructure.getInstance().hasTwoBalls)
    //                 setOpenLoop(0, 0);
    //             // indexIntake();
    //             break;
    //         case CLIMBING:
    //             mGripper.set(ControlMode.Disabled, 0);
    //             mFeeder.set(ControlMode.Disabled, 0);
    //             break;
    //         case UNJAMMING:
    //             setOpenLoop(-0.25, -0.25);
    //             // indexIntake();
    //             break;
    //         default:
    //             System.out.println("Fell through on Indexer states!");
    //         }
    }
}