/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3707.Creedence.commands.autonomous;

import org.usfirst.frc3707.Creedence.Robot;
import org.usfirst.frc3707.Creedence.pixy2API.Pixy2Line.Vector;
import org.usfirst.frc3707.Creedence.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class PixyDriveAssist extends Command {

  double centerOfPixy = 50; //one sec
  

  long lastTime;
  public double output = 0;
  public double errSum, lastErr = 0;
  double kp = 0.003;
  double ki = 0;

  private Vector[] vectors;

  public PixyDriveAssist() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.driveSubsystem);
    
  }
  

  /**
   * A method to compute PID motor power.
   * 
   * @return A double indicating how much to power a motor
   * @param input    This is the current value being received from the sensor
   * @param setpoint The desired setpoint
   */
  public double computePIDPower(double input, double setpoint) {
    /* How long since we last calculated */
    long now = System.currentTimeMillis();
    double timeChange = (double) (now - lastTime);
    /* Compute all the working error variables */
    double error = setpoint - input;
    errSum += (error * timeChange);

    /* Compute PID Output */
    output = kp * error + ki * errSum;

    /* Remember some variables for next time */
    lastErr = error;
    lastTime = now;

    return output;
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    this.vectors = Robot.m_pixy.findVectors();

    //If we dont see the line, drive normal and return
    if(this.vectors == null) {
      Robot.oi.driveByJoystick(-Robot.oi.driverController.getLeftStickXValue());
      return;
    }

    double pixyDriveAssist = 0;

    double x0 = vectors[0].getX0();
    double x1 = vectors[0].getX1();

    double x_difference = ((x0 + x1) / 2);

    if (x_difference > 3)
    {
      double errorOutput = computePIDPower(x_difference, centerOfPixy);
      pixyDriveAssist = errorOutput * 3.75; //3.75 is the found value for nice movement
    }
    Robot.oi.driveByJoystick(pixyDriveAssist);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
