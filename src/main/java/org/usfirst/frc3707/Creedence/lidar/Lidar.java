package org.usfirst.frc3707.Creedence.lidar;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class Lidar implements PIDSource {
	/*
	 * Adjust the Calibration Offset to compensate for differences in each unit.
	 * We've found this is a reasonably constant value for readings in the 25 cm to
	 * 600 cm range. You can also use the offset to zero out the distance between
	 * the sensor and edge of the robot.
	 */
	private static final int CALIBRATION_OFFSET = 0;

	private Counter counter;
	private int printedWarningCount = 5;

	/**
	 * Create an object for a LIDAR-Lite attached to some digital input on the
	 * roboRIO
	 * 
	 * @param source The DigitalInput or DigitalSource where the LIDAR-Lite is
	 *               attached (ex: new DigitalInput(9))
	 */
	public Lidar(DigitalSource source) {
		counter = new Counter(source);
		counter.setMaxPeriod(1.0);
		// Configure for measuring rising to falling pulses
		counter.setSemiPeriodMode(true);
		counter.reset();
	}

	public ArrayList<Double> distanceArray = new ArrayList<Double>();

	/**
	 * Take a measurement and return the distance in cm
	 * 
	 * @return Distance in cm
	 */
	public double getDistance() {
		distanceArray.clear();
		for (int i = 0; i < 80; i++) {
			double cm;
			/*
			 * If we haven't seen the first rising to falling pulse, then we have no
			 * measurement. This happens when there is no LIDAR-Lite plugged in, btw.
			 */
			if (counter.get() < 1) {
				if (printedWarningCount-- > 0) {
					System.out.println("LidarLitePWM: waiting for distance measurement");
				}
				return 0;
			}
			/*
			 * getPeriod returns time in seconds. The hardware resolution is microseconds.
			 * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of
			 * distance.
			 */
			cm = (counter.getPeriod() * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
			// converts cm to inches #america
			cm = cm / 2.54;

			try {
				distanceArray.add(i, cm);
			} catch (IndexOutOfBoundsException e) {
				System.out.println("Yo, Out of Bounds happened in Lidar..." + e);
			}

		}

		try {
			Collections.sort(distanceArray);
		} catch (Exception e) {
			e.printStackTrace();
		}
			double sum = 0;
		for (int i = 0; i < distanceArray.size(); i++) {
			try {
				sum += distanceArray.get(i);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		double average = sum / distanceArray.size();
		return average;
	}

	public ArrayDeque<Double> errorArrayDeque = new ArrayDeque<Double>();
// hey guys finish this later
	public double getLidarError() {
		errorArrayDeque.push(getDistance());
		if (errorArrayDeque.size() > 10) {
			errorArrayDeque.pop();

		}
		if (Math.abs(errorArrayDeque.getFirst() - errorArrayDeque.getLast()) > 5){
			return errorArrayDeque.getFirst();
		}else return getLidarError();

	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		double distance = (getDistance());
		return distance;
	}

	public boolean at(double hite) {
		double amount = 0;
		for (int x = 0; x < distanceArray.size(); x++) {
			amount += distanceArray.get(x);
		}
		amount = amount / distanceArray.size();

		if (Math.abs(hite - amount) < .5) {
			return true;
		}
		return false;
	}

}