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

	public static void setLimits(double high, double low)
	{
		max = high;
		min = low;
	}
	
	public static void setReverse(boolean reverse)
	{
		reversed = reverse;
	}
	
	public static void setPID(double P, double I, double D)
	{
		kP = P;
		kI = I;
		kD = D;
	}
	
	public static void adjustPID(double adjustP, double adjustI, double adjustD)
	{
		kP+=adjustP;
		kI+=adjustI;
		kD+=adjustD;
	}
	
	public static void setSetpoint(double target)
	{
		setpoint = target;
	}
	
	public static double computePID(double current, double timeNow)
	{
		double output;
		double error;
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
