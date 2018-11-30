//package SRF_PID;

import edu.wpi.first.wpilibj.Joystick;

public class SRF_PID { //v1.1.1
	/*	Fixed instance overwrite problem
	 *
	 *  IMPORTANT: THE DEFAULT BUTTON VALUES NEED TO BE DEFINED
	 *
	 *  Untested
	 *  Consider adding every PID into an array in a single object?
	 *  This could be done by adding a second dimension to k and doing the equals sign part of it's definition in SRF_PID
	 *  I'm not sure if that last part is acceptable yet
	 *
	 *  Also consider adding redo functionality
	 */
	
	public SRF_PID(Joystick j) {
		
	}
	
	int P = 0, I = 1, D = 2; //symbolics which can be passed into any array in which gains are stored
	double[] k = new double[3]; //the current values of each gain, the index is based on the integers above
	double errorSum = 0, lastError = 0;
	double setpoint;
	boolean reversed = false;
	double max = 1, min = -1;
	double lastTime = 0;

	double[] initialK = new double[3];
	double[] mult = new double[] {1,1,1};
	
	//this will soon serve as the cache of old values (up to 3 in the past)
	//the first value specifies which gain is being modified and the second denotes how many steps previous it was
	//0 is the previous value and then it counts up from there
	double[][] oldK = new double[3][3];
	
	//this is an array which stores how many 
	int[] stored = new int[3];
	stored[0] = -1;
	stored[1] = -1;
	stored[2] = -1;
	
	
	//these values need to be edited to the desired default values
	//they represent the index of the axis or whatever you're calling (e.g. control.getRawAxis(joyX))
	int  joyX, joyY;
	boolean reverseX = false, reverseY = false; //these will invert their respective joystick axes
	int  applyButton, cycleGainButton, undoButton, cycleModeButton; //cycle mode toggles between set, adjust and multiply(0,1,& 2)
	int currentMode = 0;//the mode that is changed by the cycleMode Button
	int currentGain = 0;
	
	//these booleans become false when their respective button is pressed and remain so until it is realeased
	//this means that rather then making a change for the entire duration you
	//hold a given button, you only make a single change when it is first pressed
	boolean letUpApply = true, letUpCycleGain = true, letUpUndo = true, letUpCycleMode = true;
	
	
	//a method that will manage the cache of previous changes
	public void updateUndo(int gain, double val)
	{
		//if the cache is empty then no shuffling of data is required
		if(stored[gain] == -1)
		{
			stored[gain] = 0;
			oldK[gain][0] = val;
			return;
		}
		//otherwise loop and rewrite starting with 1
		
		
		
			= stored[gain]
		
			
	}
	
	
	//sets the maximum and minimun power values that the PID can output
	//not having a limit may cause issues depending on what it is being inputted into
	//default values are 1 and -1
	public void setLimits(double high, double low)
	{
		max = high;
		min = low;
	}
	
	//if reversed is set to true then all future outputs from this PID will be multiplied by negative one
	public void setReverse(boolean reverse)
	{
		reversed = reverse;
	}
	
	public void setPID(double nP, double nI, double nD, boolean init)
	{
		
		k[P] = nP;
		k[I] = nI;
		k[D] = nD;
		
		if(init){
			initialK[P] = nP;
			initialK[I] = nI;
			initialK[D] = nD;
		}
	}
	
	public void adjustPID(double adjustP, double adjustI, double adjustD, boolean init)
	{
		k[P]+=adjustP;
		k[I]+=adjustI;
		k[D]+=adjustD;
		
		if(init){
			initialK[P]+=adjustP;
			initialK[I]+=adjustI;
			initialK[D]+=adjustD;
		}
	}
	
	public void setSetpoint(double target)
	{
		setpoint = target;
	}
	
	public double computePID(double current, double timeNow)
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
			output = (k[P] * error) + (k[I] * dT * errorSum) + k[D] / dT * (error - lastError); //Mo is typically not relevant in current output computation
		else
			output = (k[P] * error) + (k[I] * dT * errorSum);
		
		
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
	
	public void setMult(double nP, double nI, double nD)
	{
		mult[P] = nP;
		mult[I] = nI;
		mult[D] = nD;
	}
	
	public void adjustMult(double adjustP, double adjustI, double adjustD)
	{
		mult[P]+=adjustP;
		mult[I]+=adjustI;
		mult[D]+=adjustD;
	}
	
	//this sets new indices for the joysticks so that the joystick can be changed from it's default which is configured
	//for an xBox 360 controller
	//for the code to work proberly these two axes should be on the same joystick
	//the booleans set whether or not that axis is inverted
	public void configJoy(int x, boolean rX, int y, boolean rY)
	{
		joyX = x;
		reverseX = rX;
		joyY = y;
		reverseY = rY;
	}
	
	//this sets the indices of the buttons that are being 
	public void configButtons(int apply, int undo, int cycleGain, int cycleMode)
	{
		applyButton = apply;
		undoButton = undo;
		cycleGainButton = cycleGain;
		cycleModeButton = cycleMode;
	}
	
	//This method should be called in every loop as it is what updates these methods
	public void controlPID()
	{
		//cycleGain
		if(j.getRawButton(cycleGainButton) && letUpCycleGain)
		{
			currentGain += 1;
			letUpCycleGain = false;
		}
		else if(!j.getRawButton(cycleGainButton))
			letUpCycleGain = true;
			
		
		//cycleMode
		if(j.getRawButton(cycleModeButton) && letUpCycleMode)
		{
			currentMode += 1;
			letUpCycleMode = false;
		}
		else if(!j.getRawButton(cycleModeButton))
			letUpCycleMode = true;
		
		//undo
		
		
		//joystick
		
		
		//apply
	}
}
