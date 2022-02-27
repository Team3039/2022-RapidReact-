package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;

public class Indexer extends SubsystemBase {
    private static Indexer mInstance = null;
    
    private static final double kZoomingVelocity = 80.;
    private static final double kPassiveIndexingVelocity = 80.0;

    private static final double kJamCurrent = 150.0;
    private double mLastCurrentSpikeTime = 0.0;
    private static final double kCurrentIgnoreTime = 1.0; 

    public enum IndexerState {
        IDLE, PASSIVE_INDEXING,  ACTIVE_INDEXING, CLIMBING, HELLA_ZOOMING, CLIMB,
    }

    private final TalonFX mGripper;
    private final TalonFX mFeeder;

    private final DigitalInput mFeederGate;
    private final DigitalInput mGripperGate;

    private IndexerState mState = IndexerState.IDLE;
    private boolean mBackwards = false;

    private boolean hasOneBall;
    private boolean hasTwoBalls;
    private boolean isJamming;

    public Indexer() {
        mFeeder = new TalonFX(0);
        mGripper = new TalonFX(0);

        mFeederGate = new DigitalInput(0);
        mGripperGate = new DigitalInput(0);

        mFeeder.changeMotionControlFramePeriod(255);
        mFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);
        mFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 125);

        mFeeder.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mFeeder.enableVoltageCompensation(true);
    }
    
    public synchronized IndexerState getState() {
        return mState;
    }

    public synchronized static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public synchronized void setOpenLoop(
      double gripperOutput,
      double feederOutput) {
       mGripper.set(ControlMode.PercentOutput, gripperOutput);
       mFeeder.set(ControlMode.PercentOutput, feederOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
        mFeeder.setNeutralMode(NeutralMode.Brake);
    }

    public boolean indexIntake(boolean isActive) {
        if (isActive) {
            switch (Indexer.getInstance().getState()) {
                case ACTIVE_INDEXING:
                    if (!this.hasTwoBalls)
                        Intake.getInstance().setState(IntakeState.INTAKING);
                    else if (isJamming)
                        Intake.getInstance().setState(IntakeState.OUTTAKING);
                    else
                        Intake.getInstance().setState(IntakeState.IDLE);
                default:
                    break;
            }
            return true;
        }
        return false;
    }

    public synchronized void setBackwardsMode(boolean backwards) {
        mBackwards = backwards;
    }

    public void setState(IndexerState wanted_state) {
        final IndexerState prev_state = mState;
        mState = wanted_state;
        

        if (mState != prev_state && mState == IndexerState.PASSIVE_INDEXING) {
            mBackwards = !mBackwards;
        }

        if (mState != prev_state && mState == IndexerState.ACTIVE_INDEXING) {
            mFeeder.configClosedloopRamp(0.2, 0);
        } else if (mState != prev_state) {
            mFeeder.configClosedloopRamp(0.0, 0);
        }
    }

    @Override
    public void periodic() {
        switch (mState) {
            case IDLE:
                setOpenLoop(0, 0);
                break;
            case PASSIVE_INDEXING:
                setOpenLoop(0.5, 0.5);
                break;
            case ACTIVE_INDEXING:
                

                if (!hasOneBall && !hasTwoBalls)
                    setOpenLoop(0.75, 0.75);
                if (hasOneBall)
                    setOpenLoop(0.25, 0);
                if (hasTwoBalls)
                    setOpenLoop(0, 0);
                break;
            case CLIMB:
                mGripper.set(ControlMode.Disabled, 0);
                mFeeder.set(ControlMode.Disabled, 0);
                break;
            default:
                System.out.println("Fell through on Indexer states!");
            }
    }
}