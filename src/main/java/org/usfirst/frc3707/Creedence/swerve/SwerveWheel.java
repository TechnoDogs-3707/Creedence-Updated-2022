package org.usfirst.frc3707.Creedence.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveWheel {
    private PIDController rotation_pid;
    private MotorController rotation;
    private MotorController speed;
    private AnalogPotentiometer gyro;
    private double offset;
    private boolean enabled = true;

    public SwerveWheel(PIDController rotation_pid, AnalogPotentiometer gyro,  MotorController rotation, MotorController speed, double offset) {
        System.out.println("wheel Initialized");
        this.rotation_pid = rotation_pid;
        this.gyro = gyro;
        this.rotation = rotation;
        this.speed = speed;
        this.offset = offset;
    }

    /**
     * Drives a single swerve wheel.
     * 
     * @param newSpeed The speed at which to drive the drive wheel
     * @param newAngle The angle at which to position the drive wheel
     */
    public void drive(double newSpeed, double newAngle) {
        updateSpeed(newSpeed);
        updateRotation(newAngle);
    }

    /**
     * Update the speed at which to drive the wheel
     * 
     * @param newSpeed The speed at which to drive the drive wheel
     */
    public void updateSpeed(double newSpeed) {
        if(enabled) {
            speed.set(newSpeed);
        }
    }

    /**
     * Update the angle at which to position the drive wheel
     * 
     * @param newAngle The angle at which to position the drive wheel
     */
    public void updateRotation(double newAngle) {
        newAngle = newAngle + offset;

        if (newAngle < 0) {
            rotation_pid.setSetpoint(360 - (newAngle * -1));
        } else if (newAngle > 360) {
            rotation_pid.setSetpoint(newAngle - 360);
        } else {
            rotation_pid.setSetpoint(newAngle);
        }

        double motor_output = rotation_pid.calculate(gyro.get());

        rotation.set(motor_output);
    }

    public void disableRotation() {
        rotation.disable();
        enabled = false;
    }

    public void enableRotation() {
        enabled = true;
    }
}
