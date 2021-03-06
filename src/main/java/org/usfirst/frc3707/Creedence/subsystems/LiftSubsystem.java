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
import org.usfirst.frc3707.Creedence.commands.lift.liftUpAndDownCommand;
import org.usfirst.frc3707.Creedence.lidar.Lidar;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class LiftSubsystem extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private PWMVictorSPX elevator = new PWMVictorSPX(Constants.Elevator);
    public Lidar lidarCrab = new Lidar(new DigitalInput(Constants.lidar1));
    public PIDController liftController = new PIDController( 0.2, 0.0, 0.0);

    // public PIDController liftController = new PIDController( 0.2, 0.0, 0.0, lidarCrab, elevator);
    // public PIDController liftController = new PIDController(.2, 0, 0, lidarCrab,
    // elevator, .02);

    //solinoid

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public LiftSubsystem() {
        liftController.setTolerance(0.5);
        //SmartDashboard.putNumber("Lift Height Value", 40);

        
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new liftUpAndDownCommand());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    /**
     * This method operates the teleop control of the game piece mechanism
     */
    public void liftUpAndDown() { 
        if ((Robot.oi.operatorController.getLeftStickYValue() > .3 || Robot.oi.operatorController.getLeftStickYValue() < -.3)) {
            elevator.set(Robot.oi.operatorController.getLeftStickYValue());
        } else {
            holdLift();
        }
    }

    public void liftUp() {
        elevator.set(-1);
    }

    public void liftDown() {
        elevator.set(1);
    }

    public void holdLift() {
        elevator.set(0.05);
    }

    public void liftUpOrDown(double power) {
        elevator.set(power);
    }

    public double getTargetHeight() {
        return SmartDashboard.getNumber("Lift Height Value", 40);
    }

    public double getLiftHeight() {
        return lidarCrab.getDistance();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("lidar #", lidarCrab.getDistance());
        //SmartDashboard.putNumber("POV", Robot.oi.joystick2.getPOV());
        
        // Put code here to be run every loop
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
