package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LiveData;
import frc.robot.utils.Logger;

import static frc.robot.subsystems.Superstructure.SuperstructureState.*;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    // private final HPIntake hpIntake;

    private SuperstructureState systemState;
    private SuperstructureState requestedSystemState;

    private Timer timer;

    private LiveData systemStateData, requestedSystemStateData, algaeIndex;

    private boolean isManualControl, hasJustRemovedAlgae;
    private double startedOuttakingRemovedAlgaeTime;
    
  

    // flag used for align
    //private ScoringFlag scoringFlag;

   // private double L4offset;

    public enum SuperstructureState {
        STOW,
        HP_INTAKE,
        ALGAE_GROUND_INTAKE,
        ALGAE_LOLLIPOP_INTAKE,
        PRESTAGE,
        L1_PREP,
        L2_PREP,
        L3_PREP,
        L4_PREP,
        L1_SCORE,
        L2_SCORE,
        L3_SCORE,
        L4_SCORE,
        BARGE_PRESTAGE,
        BARGE_PREP,
        BARGE_SCORE,
        PROCESSOR_PREP,
        PROCESSOR_SCORE,
        REEF1_ALGAE_INTAKE,
        REEF2_ALGAE_INTAKE,
        EJECT_ALGAE,
        EJECT_CORAL,
        REMOVING_ALGAE,
        CLIMB
    }

    public enum ScoringFlag {
        L1FLAG,
        L2FLAG,
        L3FLAG,
        L4FLAG
    }

    public Superstructure() {
        systemState = STOW;
        requestedSystemState = STOW;

        timer = new Timer();

        systemStateData = new LiveData(systemState.toString(), "System State"); 
        requestedSystemStateData = new LiveData(requestedSystemState.toString(), "Requested System State"); 

    }
    public static Superstructure getInstance() {
        if (superstructure == null) {
            superstructure = new Superstructure();
        }
        return superstructure;
    }

    public void requestState(SuperstructureState request) {
        requestedSystemState = request;
    }

    public SuperstructureState getCurrentState() {
        return systemState;
    }

    public SuperstructureState getRequestedState() {
        return requestedSystemState;
    }

    public void setManualControl(boolean isTrue){
        isManualControl = isTrue;
    }
    public boolean isClimbState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isClimbState'");
    }
}
