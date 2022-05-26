package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;
import javax.swing.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    public enum IndexerState {
        IDLE, SHOOTING, INDEXING, CLIMBING, HELLA_ZOOMING, UNJAMMING,
    }

    public final TalonSRX mFirstStageMaster;
    public final TalonSRX mSecondStageMaster;

    public final DigitalInput mFirstStageGate;
    public final DigitalInput mSecondStageGate;

    private IndexerState state = IndexerState.IDLE;

    public edu.wpi.first.wpilibj.Timer mTimer = new edu.wpi.first.wpilibj.Timer();

    public boolean hasOneBall;
    public boolean hasTwoBalls;
    public boolean isFeeding;

    public Indexer() {
        mFirstStageMaster = new TalonSRX(Constants.Ports.FIRST_STAGE);
        mSecondStageMaster = new TalonSRX(Constants.Ports.SECOND_STAGE);

        mFirstStageGate = new DigitalInput(Constants.Ports.FIRST_STAGE_GATE);
        mSecondStageGate = new DigitalInput(Constants.Ports.SECOND_STAGE_GATE);

        mFirstStageMaster.setInverted(false);

        mFirstStageMaster.setNeutralMode(NeutralMode.Brake);
        mSecondStageMaster.setNeutralMode(NeutralMode.Brake);

        mFirstStageMaster.configPeakCurrentLimit(40);
        mFirstStageMaster.configContinuousCurrentLimit(35);

        mSecondStageMaster.configPeakCurrentLimit(40);
        mSecondStageMaster.configContinuousCurrentLimit(35);

        mTimer.start();

        // mFirstStageMaster.configSupplyCurrentLimit(currLimitConfigs)
    }

    public synchronized IndexerState getState() {
        return state;
    }

    public void setState(IndexerState wanted_state) {
        state = wanted_state;
    }

    public void setOpenLoop(double firstStageOuput, double secondStageOutput) {
        mFirstStageMaster.set(ControlMode.PercentOutput, firstStageOuput);
        mSecondStageMaster.set(ControlMode.PercentOutput, secondStageOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    @Override
    public void periodic() {
        hasOneBall = !mSecondStageGate.get();
        hasTwoBalls = hasOneBall && !mFirstStageGate.get();

        // SmartDashboard.putString("Indexer State", String.valueOf(getState()));

        switch (state) {
            case HELLA_ZOOMING:
                System.out.println("Zoomies");
            case IDLE:
                setOpenLoop(0, 0);
                break;
            case SHOOTING:
                if (mTimer.hasElapsed(0.5))
                    setOpenLoop(0.6, 0.75);
                else
                    setOpenLoop(0, 0.75);
                break;
            case INDEXING:
                isFeeding = false;
                if (!hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.95, 0.40);
                } else if (hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.95, 0);
                } else if (hasTwoBalls) {
                    setOpenLoop(0, 0);
                }
                break;
            case UNJAMMING:
                setOpenLoop(-0.90, -0.45);
                break;
            case CLIMBING:
                mFirstStageMaster.set(ControlMode.Disabled, 0);
                mSecondStageMaster.set(ControlMode.Disabled, 0);
                break;
            default:
                System.out.println("Fell through on Indexer states!");
        }
    }
}