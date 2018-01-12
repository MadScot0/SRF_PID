
public class SRF_PID {

	static double kP, kI, kD;
	static double errorSum = 0, lastError = 0;
	static double setpoint;
	static boolean reversed = false;

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
	
	public static double computePID(double current)
	{
		double output;
		double error;
		
		error = setpoint - current;
		errorSum+=error;
		output = (kP * error) + (kI * errorSum) + kD * (error - lastError); //Mo is typically not relevant in current output computation
		
		if(reversed)
			output*=-1;
		
		lastError = error; //Go here for math basis
							//https://library.automationdirect.com/methods-behind-pid-loop-control/
		return output;
	}
}
