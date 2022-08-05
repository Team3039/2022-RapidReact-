package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MathUtils;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Indexer extends SubsystemBase {

    public enum IndexerState {
        IDLE, SHOOTING, INDEXING, CLIMBING, HELLA_ZOOMING, UNJAMMING,
    }

    public final TalonSRX firstStage;
    public final TalonSRX secondStage;

    public final DigitalInput firstStageGate;
    public final DigitalInput secondStageGate;

    private IndexerState state = IndexerState.IDLE;

    public edu.wpi.first.wpilibj.Timer mTimer = new edu.wpi.first.wpilibj.Timer();

    public boolean hasOneBall;
    public boolean hasTwoBalls;
    public boolean isFeeding;

    public Indexer() {
        firstStage = new TalonSRX(Constants.Ports.FIRST_STAGE);
        secondStage = new TalonSRX(Constants.Ports.SECOND_STAGE);

        firstStageGate = new DigitalInput(Constants.Ports.FIRST_STAGE_GATE);
        secondStageGate = new DigitalInput(Constants.Ports.SECOND_STAGE_GATE);

        firstStage.setInverted(false);

        firstStage.setNeutralMode(NeutralMode.Brake);
        secondStage.setNeutralMode(NeutralMode.Brake);

        // firstStage.configPeakCurrentLimit(40);
        // firstStage.configContinuousCurrentLimit(35);

        // secondStage.configPeakCurrentLimit(40);
        // secondStage.configContinuousCurrentLimit(35);

        mTimer.start();
    }

    public synchronized IndexerState getState() {
        return state;
    }

    public void setState(IndexerState wanted_state) {
        state = wanted_state;
    }

    public void setOpenLoop(double firstStageOuput, double secondStageOutput) {
        firstStage.set(ControlMode.PercentOutput, firstStageOuput);
        secondStage.set(ControlMode.PercentOutput, secondStageOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    @Override
    public void periodic() {
        hasOneBall = !secondStageGate.get();
        hasTwoBalls = hasOneBall && !firstStageGate.get();

        SmartDashboard.putNumber("Second Stage Stator Current", secondStage.getStatorCurrent());
        SmartDashboard.putNumber("Second Stage Supply Current", secondStage.getSupplyCurrent());

        // SmartDashboard.putString("Indexer State", String.valueOf(getState()));

        
        switch (state) {
            case HELLA_ZOOMING:
                System.out.println("Zoomies");
            case IDLE:
                mTimer.stop();
                mTimer.reset();
                setOpenLoop(0, 0);
                break;
            case SHOOTING:
              if (Shooter.isAtSetPoint && Turret.isAtTargetPosition) {
                mTimer.start();
                if (mTimer.hasElapsed(1.25)) // Was 2 seconds
                    setOpenLoop(0.55, .9);
                else
                    setOpenLoop(0, .9);
              } 
                break;
            case INDEXING:
                isFeeding = false;
                if (!hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.9, 0.50);
                } else if (hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.70, 0); // Was (.60, 0)
                } else if (hasTwoBalls) {
                    setOpenLoop(0, 0);
                }
                break;
            case UNJAMMING:
                setOpenLoop(-0.90, -0.45);
                break;
            case CLIMBING:
                firstStage.set(ControlMode.Disabled, 0);
                secondStage.set(ControlMode.Disabled, 0);
                break;
            default:
                System.out.println("Fell through on Indexer states!");
        }
    }
}