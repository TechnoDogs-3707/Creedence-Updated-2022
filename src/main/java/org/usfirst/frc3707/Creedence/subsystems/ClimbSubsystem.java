// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc3707.Creedence.subsystems;

import org.usfirst.frc3707.Creedence.Robot;
import org.usfirst.frc3707.Creedence.Configuration.Constants;
import org.usfirst.frc3707.Creedence.commands.climb.climbBarMove;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ClimbSubsystem extends Subsystem {

    private PWMVictorSPX climbBar = new PWMVictorSPX(Constants.ClimbSystem.getClimbBar());
    //private PWMVictorSPX climbGrab = new PWMVictorSPX(Constants.ClimbSystem.getClimbGrab());
    private PWMVictorSPX climbPullForward = new PWMVictorSPX(Constants.ClimbSystem.getClimbPullForward());

    public ClimbSubsystem() {
        climbBar.stopMotor();
        climbPullForward.set(0);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new climbBarMove());
    }

    public void controlBar(){
        if(Robot.oi.operatorController.getRightStickYValue() > 0.2 || Robot.oi.operatorController.getRightStickYValue() < -0.2) {
            climbBar.set(Robot.oi.operatorController.getRightStickYValue());
        }
        else {
            climbBar.set(0);
        }

    }
    public void stopClimb() {
        climbBar.set(0);
    }
    
    public void startVacuum(){
        climbPullForward.set(0.4);
    }
    public void stopVacuum(){
        climbPullForward.set(0);
    }

    /**
     * This method operates the teleop control of the game piece mechanism
     */
    

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }
}
