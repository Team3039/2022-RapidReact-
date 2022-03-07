package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    public enum IndexerState {
        IDLE, SHOOTING, INDEXING, CLIMBING, HELLA_ZOOMING, UNJAMMING,
    }

    public final TalonSRX firstStage;
    public final TalonSRX secondStage;

    public final DigitalInput secondStageGate;
    public final DigitalInput firstStageGate;

    private IndexerState state = IndexerState.IDLE;

    private boolean hasOneBall;
    private boolean hasTwoBalls;
    public boolean isFeeding;
  

    public Indexer() {
        firstStage = new TalonSRX(Constants.RobotMap.firstStage);
        secondStage = new TalonSRX(Constants.RobotMap.secondStage);

        firstStageGate = new DigitalInput(Constants.RobotMap.firstStageGate);
        secondStageGate = new DigitalInput(Constants.RobotMap.secondStageGate);

        firstStage.setNeutralMode(NeutralMode.Brake);
        secondStage.setNeutralMode(NeutralMode.Brake);
    }

    public synchronized IndexerState getState() {
        return state;
    }

    public void setState(IndexerState wanted_state) {
        state = wanted_state;
    }

    public void setOpenLoop(double firstStageOutput, double secondStageOutput) {
       firstStage.set(ControlMode.PercentOutput, firstStageOutput);
       secondStage.set(ControlMode.PercentOutput, secondStageOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    @Override
    public void periodic() {
        hasOneBall = !secondStageGate.get();
        hasTwoBalls = hasOneBall && !firstStageGate.get();

        switch (state) {
            case HELLA_ZOOMING:
                System.out.println("Zoomies");
            case IDLE:
                setOpenLoop(0, 0);
                break;
            case SHOOTING:
                isFeeding = true;
                setOpenLoop(0.6, 0.75);
                break;
            case INDEXING:
                isFeeding = false;
                if (!hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.60, 0.40); 
                }
                else if (hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.60, 0);
                }
                else if (hasTwoBalls) {
                    setOpenLoop(0, 0);
                }
                break;
            case CLIMBING:
                firstStage.set(ControlMode.Disabled, 0);
                secondStage.set(ControlMode.Disabled, 0);
                break;
            case UNJAMMING:
                    setOpenLoop(-0.25, -0.25);
                break;
            default:
                System.out.println("Fell through on Indexer states!");
            }
    }
}