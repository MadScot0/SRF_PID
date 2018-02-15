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
	static double max = 1, min = -1;
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
		
		if(timeNow == 0)
			errorSum = 0;
		
		error = setpoint - current;
		errorSum+=error;
		//System.out.println("("+kP+"*"+error+")+("+kI+"*"+dT+"*"+errorSum+")+"+kD+"/"+dT+"*("+error+"-"+lastError+")");
		if(dT != 0)
			output = (kP * error) + (kI * dT * errorSum) + kD / dT * (error - lastError); //Mo is typically not relevant in current output computation
		else
			output = (kP * error) + (kI * dT * errorSum);
		
		
		if(reversed)
			output*=-1;
		
		if(output > max)
			output = max;
		else if(output < min)
			output = min;
		//System.out.println("="+output);
		lastError = error; //Go here for math basis
							//https://library.automationdirect.com/methods-behind-pid-loop-control/
		return output;
	}
}
