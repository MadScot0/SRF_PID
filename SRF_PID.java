package org.usfirst.frc.team3826.robot;
public class SRF_PID { //v1.1
	/*	Added delta time funcionality
	 *  Untested
	 * 
	 * 
	 * 
	 */

	static double kP, kI, kD;
	static double errorSum = 0, lastError = 0;
	static double setpoint;
	static boolean reversed = false;
	static double max, min;
	static double lastTime = 0;

	public static void setLimits(double high, double low) //this method sets the output limits for this PID object
	{
		max = high;
		min = low;
	}
	
	public static void setReverse(boolean reverse) //method to reverse output values
	{
		reversed = reverse;
	}
	
	public static void setPID(double P, double I, double D) //method to set values of constants in the PID loop
	{
		kP = P;
		kI = I;
		kD = D;
	}
	
	public static void adjustPID(double adjustP, double adjustI, double adjustD) //method so that people can incrementally adjust
	{									//PID constants
		kP+=adjustP;
		kI+=adjustI;
		kD+=adjustD;
	}
	
	public static void setSetpoint(double target) //set the setpoint that the PID will approach; the vagueness allows for many
	{						//applications (e.g. speed, position, etc.)
		setpoint = target;
	}
	
	public static double computePID(double current, double timeNow) //the method to compute what the output of the PID control should
	{								//should be, current is the value of the sensor that your setpoint
		double output;						//is compared to; timeNow just needs to be a refernece to a steady 
		double error;						//change, DON'T PAUSE THE TIMER
		double dT = timeNow - lastTime;
		lastTime = timeNow;
		
		error = setpoint - current;
		errorSum+=error;
		output = (kP * error) + (kI * dT * errorSum) + kD * dT * (error - lastError); //Mo is typically not relevant in current output computation
		
		if(reversed)
			output*=-1;
		
		if(output > max)
			output = max;
		else if(output < min)
			output = min;
		
		lastError = error; //Go here for math basis
							//https://library.automationdirect.com/methods-behind-pid-loop-control/
		return output;
	}
}
