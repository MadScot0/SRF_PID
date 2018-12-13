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
	 *  
	 *  CHECK NOTES above apply in ControlPID()
	 */
	
	public SRF_PID(Joystick js) {
		j = js;
	}
	
	Joystick j;
	
	int P = 0, I = 1, D = 2; //symbolics which can be passed into any array in which gains are stored
	double[] k = new double[3]; //the current values of each gain, the index is based on the integers above
	double errorSum = 0, lastError = 0;
	double setpoint;
	boolean reversed = false;
	double max = 1, min = -1;
	double lastTime = 0;
	int inverser = 1; //Preset multiplied by this to switch between adding and subtracting

	double[] = new double[3];
	double[] mult = new double[] {1,1,1};
	
	final int storable = 3;//the variable that defines undo array sizes
	//this will soon serve as the cache of old values (up to 3 in the past)
	//the first value specifies which gain is being modified and the second denotes how many steps previous it was
	//0 is the previous value and then it counts up from there
	double[][] oldK = new double[3][storable];
	
	//this is an array which stores which cell was most recently written to in each previous value cache
	//this most recent value represents the current value of the respective gain
	//-1 is the default value for each array and signals that it hasn't been used yet
	int[] mostRecent = {-1,-1,-1};
	
	//this array stores how many steps back you've gone (using this value will give us the ability to easily incorpoarate redo later on
	//a value of 0 means that this gain hasn't been undone since the code was adjusted
	int[] mostRecentUndo {0,0,0}
	
	
	//these values need to be edited to the desired default values
	//they represent the index of the axis or whatever you're calling (e.g. control.getRawAxis(joyX))
	int  joyX = 1, joyY = 2;
	boolean reverseX = false, reverseY = false; //these will invert their respective joystick axes
	int  applyButton = 1, cycleGainButton = 2, undoButton = 3, cycleModeButton = 4; //cycle mode toggles between set, adjust and multiply(0,1,& 2)
	int preset10Button = 5, preset50Button = 6, preset100Button = 7, inversePresetButton = 8; //preset buttons which change mult values by plus or minus the number on the end, inverse switches the sign of the preset (*1 or *-1)
	int currentMode = 0;//the mode that is changed by the cycleMode Button (set = 0, adjust = 1, multiply = 2)
	int currentGain = 0;
	
	//these booleans become false when their respective button is pressed and remain so until it is realeased
	//this means that rather then making a change for the entire duration you
	//hold a given button, you only make a single change when it is first pressed
	boolean letUpApply = true, letUpCycleGain = true, letUpUndo = true, letUpCycleMode = true, letUpPreset10 = true, letUpPreset50 = true, letUpPreset100 = true, letUpInversePreset = true;
	
	
	//a method that will manage the cache of previous changes
	public void updateUndo(int gain, double val)
	{
		//circular arrays
		int tempIndex = mostRecent[gain]-mostRecentUndo[gain]; //the value that stores the cell that the new value is being written to in the cache
		
		//reduce it to keep it with in the index of the array
		while(tempIndex > 2)
			tempIndex-=3;
		
		oldK[gain][tempIndex] = val;
		mostRecent[gain] = tempIndex;
		mostRecentUndo[gain] = -1;//it defines that there are no recent undos and begins to overwrite the old values
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
	
	//sets a specific gain
	public void setGain(int gain, double value)
	{
		k[gain] = value;
	}
	
	//sets the value of all three gains
	public void setPID(double nP, double nI, double nD)
	{
		
		k[P] = nP;
		k[I] = nI;
		k[D] = nD;
		
	}
	
	//adds or subtracts to all three gains
	public void adjustPID(double adjustP, double adjustI, double adjustD)
	{
		k[P]+=adjustP;
		k[I]+=adjustI;
		k[D]+=adjustD;
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
		//each of the button must be physically released before their code can trigger again
		
		//cycleGain
		if(j.getRawButton(cycleGainButton) && letUpCycleGain)
		{
			currentGain += 1;
			if(currentGain == 3)
				currentGain = 0;
			letUpCycleGain = false;
		}
		else if(!j.getRawButton(cycleGainButton))
			letUpCycleGain = true;
			
		
		//cycleMode
		if(j.getRawButton(cycleModeButton) && letUpCycleMode)
		{
			currentMode += 1;
			if(currentMode == 3)
				currentMode = 0;
			letUpCycleMode = false;
		}
		else if(!j.getRawButton(cycleModeButton))
			letUpCycleMode = true;
		
		
		//undo
		//this will only trigger if you have updated a value for this gain
		//it also limits undos so that you can't make a full loop
		if(j.getRawButton(undoButton) && letUpUndo && mostRecent[currentGain] > -1 && mostRecentUndo[currentGain] < storable - 1)
		{
			mostRecentUndo[currentGain]++;//sets the value to the one that has most recently been written to minus the number
							//of consecutive undos
			setValue(currentGain, oldK[currentGain][mostRecent[currentGain]-mostRecentUndo[currentGain]]);
		}
			
			
		//joystick (Dial)
		
		
		//apply - applies value right and then updateUndo is called (Dial + presets)
		//Should we only be able to apply changes to one gain at a time?
		//This is written on the assumption that we can change multiple values at a time, then hit apply.
		//Note to above line - mostly because if we can only change one value at a time why is mult an array? Also if we
		//are going to do this we should cahnge updateUndo to do all three at once
		if(j.getRawButton(applyButton) && letUpApply) {
			k[P] *= mult[P];
			k[I] *= mult[I];
			k[D] *= mult[D];
			updateUndo(P,k[P]);
			updateUndo(I,k[I]);
			updateUndo(D,k[D]);
		}
		
		//preset adjustments - ADD MORE BUTTONS FOR THIS - also it requires update undo after the update is applied
		if(j.getRawButton(inversePresetButton) && letUpInversePreset) {
			inverser *= -1;
		}
		
		if(j.getRawButton(preset10Button) && letUpPreset10) {
			mult[currentGain] += .1 * inverser;
		}
		
		if(j.getRawButton(preset50Button) && letUpPreset50) {
			mult[currentGain] += .5 * inverser;
		}
		
		if(j.getRawButton(preset100Button) && letUpPreset100) {
			mult[currentGain] += 1 * inverser;
		}
		
	}
}
