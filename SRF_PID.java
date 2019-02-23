///package org.usfirst.frc.team3826.robot;
package frc.robot;
//package SRF_PID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRF_PID { //v1.1.3
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
	 *  What should be called instead of setValues() in undo of controlPID() as setValues no longer exists?
	 *  
	 *  Test Dial
	 */
	
	public SRF_PID(Joystick js, double kP, double kI, double kD) {
		j = js;
		oldK[P][0] = kP;
		oldK[I][0] = kI;
		oldK[D][0] = kD;
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

	double[] mult = new double[] {1,1,1};
	
	final int storable = 3;//the variable that defines undo array sizes
	//this will soon serve as the cache of old values (up to 3 in the past)
	//the first value specifies which gain is being modified and the second denotes how many steps previous it was
	//0 is the previous value and then it counts up from there
	double[][] oldK = new double[3][storable];
	
	//this is an array which stores which cell was most recently written to in each previous value cache
	//this most recent value represents the current value of the respective gain
	//-1 is the default value for each array and signals that it hasn't been used yet
	int[] mostRecent = {0,0,0};
	boolean[] isUpdated = {false,false,false};
	
	//this array stores how many steps back you've gone (using this value will give us the ability to easily incorpoarate redo later on
	//a value of 0 means that this gain hasn't been undone since the code was adjusted
	int[] mostRecentUndo = {0,0,0};
	
	
	//these values need to be edited to the desired default values
	//they represent the index of the axis or whatever you're calling (e.g. control.getRawAxis(joyX))
	int  joyX = 1, joyY = 2;
	boolean reverseX = false, reverseY = false; //these will invert their respective joystick axes
	int  applyButton = 1, cycleGainButton = 2, undoButton = 3, cycleModeButton = 4, multClearButton = 10;//cycle mode toggles between set, adjust and multiply(0,1,& 2)
	int preset10Button = 5, preset50Button = 6, preset100Button = 7, inversePresetButton = 8; //preset buttons which change mult values by plus or minus the number on the end, inverse switches the sign of the preset (*1 or *-1)
	int currentMode = 0;//the mode that is changed by the cycleMode Button (set = 0, adjust = 1, multiply = 2)
	int currentGain = 0;//The gain that is changed by the cycleGain Button (ie. P, I, or D)
	
	//these booleans become false when their respective button is pressed and remain so until it is realeased
	//this means that rather then making a change for the entire duration you
	//hold a given button, you only make a single change when it is first pressed
	boolean letUpApply = true, letUpCycleGain = true, letUpUndo = true, letUpCycleMode = true, letUpPreset10 = true, letUpPreset50 = true, letUpPreset100 = true, letUpInversePreset = true, letUpClearMult = true;
	
	//Dial variables
	double slope,xCoor,yCoor,finDegree;	//slope or hypotenuse of angle Theta in unit circle,
									//X-Coordinate of intersection of the line created by the hypotenuse of angel theta and unit circle,
									//y-Coordinate of intersection of the line created by the hypotenuse of angel theta and unit circle,
									//Starts as the Arcsine or inverse sine of yCoor then becomes the final angle in degrees and finally the percentage based on the angle in degrees
	float dead = .25f;		//Size of dead band around joystick    range: 0-1
	int stickRotations = 0;	//The number of times the stick as completed a full rotation 
	int lastQuadrant;		//last quadrant of the unit circle the stick was in, Arranged like this: 	2 | 1
	int currentQuadrant;	//The Current quadrant the stick is in									   ---|---
							//																			3 | 4
	double dg;
	//a method that will manage the cache of previous changes
	public void updateUndo(int gain, double val)
	{
		mostRecent[gain]++;
		//circular arrays
		int tempIndex = mostRecent[gain]-mostRecentUndo[gain]; //the value that stores the cell that the new value is being written to in the cache
		
		//reduce it to keep it with in the index of the array
		while(tempIndex > 2)
			tempIndex-=3;
		
		oldK[gain][tempIndex] = val;
		mostRecent[gain] = tempIndex;
		mostRecentUndo[gain] = 0;//it defines that there are no recent undos and begins to overwrite the old values
			//this was -1 but I'm pretty sure it should be 0
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
	public void configButtons(int apply, int clear, int undo, int cycleGain, int cycleMode)
	{
		applyButton = apply;
		undoButton = undo;
		cycleGainButton = cycleGain;
		cycleModeButton = cycleMode;
		multClearButton = clear;
	}

	public void configPresetButtons(int inv, int preset10, int preset50, int preset100)
	{
		inversePresetButton = inv;
		preset10Button = preset10;
		preset50Button = preset50;
		preset100Button = preset100;
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
		if(j.getRawButton(undoButton) && letUpUndo && isUpdated[currentGain] && mostRecentUndo[currentGain] < storable - 1)
		{
			mostRecentUndo[currentGain]++;//sets the value to the one that has most recently been written to minus the number
										  //of consecutive undos
			setGain(currentGain, oldK[currentGain][mostRecent[currentGain]-mostRecentUndo[currentGain]]);
			letUpUndo = false;
		} else if(!j.getRawButton(undoButton)) {
			letUpUndo = true;
		}
			
			
		//joystick (Dial) - Left Stick
		int axisy = 1;	//I have no idea what the axis number is so im putting this in as a place holder(Replace all) and so it can be changed quickly
		int axisx = 0;	//same as above
		if(j.getRawAxis(axisx) > dead || j.getRawAxis(axisx) < dead*-1 || j.getRawAxis(axisy) > dead || j.getRawAxis(axisy) < dead*-1) {
			
			//This converts any point that doesn't fall on the border of the unit Circle into a point on it by finding
			//the intersection of the hypotenuse of the angle created and the circle
			if(j.getRawAxis(axisx) == 0 && -j.getRawAxis(axisy) > 0 ) {
				xCoor = 0;
				yCoor = 1;
			} else if(j.getRawAxis(axisx) == 0 && -j.getRawAxis(axisy) < 0 ) {
				xCoor = 0;
				yCoor = -1;
			} else {
				slope = (-j.getRawAxis(axisy))/j.getRawAxis(axisx);
				xCoor = 1/(Math.sqrt(Math.pow(slope,2) + 1));
				yCoor = xCoor * slope;
				xCoor = j.getRawAxis(0);
			}
			//finds angle in degrees using yCoor
			finDegree = Math.toDegrees(Math.asin(yCoor));
			dg = finDegree;
			//determines what quadrant the stick is in
			if (finDegree > 0 && xCoor > 0) {
				finDegree +=  270;
				currentQuadrant = 1;
			}else if(finDegree < 0 && xCoor < 0) {
				finDegree = (90+finDegree);
				currentQuadrant = 2;
			} else if(finDegree > 0 && xCoor <0) {
				finDegree = 90 + finDegree;
				currentQuadrant = 3;
			} else if(finDegree < 0) {
				finDegree = 270 + finDegree;
				currentQuadrant = 4;
			}
			//updates stickRotations
			if(lastQuadrant == 1 && currentQuadrant == 2)
				stickRotations++;
			else if(lastQuadrant == 2 && currentQuadrant == 1)
				stickRotations--;
			lastQuadrant = currentQuadrant;
			
			//changes finDegree from degrees to the final percentage
			if(stickRotations >= 0)
				finDegree = finDegree/3.6 + 100*stickRotations;
			else
				finDegree /= 3.6*Math.pow(10,stickRotations*-1);
			
			mult[currentGain] = finDegree/100;
			
		} else if(lastQuadrant != 0) {
			lastQuadrant = 0;
		}
		
		//apply - applies value right and then updateUndo is called (Dial + presets)
		if(j.getRawButton(applyButton) && letUpApply) {
			errorSum = 0;
			isUpdated[currentGain] = true;
			k[currentGain] *= mult[currentGain];
			updateUndo(currentGain,k[currentGain]);
			letUpApply = false;
		} else if(!j.getRawButton(applyButton)) {
			letUpApply = true;
		}
		
		//Clears the value in the currently selected gains place in mult[] - Right Stick button
		if(j.getRawButton(multClearButton) && letUpClearMult) {
			mult[currentGain] = 0;	
			letUpClearMult = false;
		} else if(!j.getRawButton(multClearButton)) {
			letUpClearMult = true;
		}
		
		//preset adjustments - ADD MORE BUTTONS FOR THIS - also it requires update undo after the update is applied
		if(j.getRawButton(inversePresetButton) && letUpInversePreset) {
			inverser *= -1;
			letUpInversePreset = false;
		} else if(!j.getRawButton(inversePresetButton)) {
			letUpInversePreset = true;
		}
		
		if(j.getRawButton(preset10Button) && letUpPreset10) {
			mult[currentGain] += .1 * inverser;
			letUpPreset10 = false;
		} else if(!j.getRawButton(preset10Button)) {
			letUpPreset10 = true;
		}
		
		if(j.getRawButton(preset50Button) && letUpPreset50) {
			mult[currentGain] += .5 * inverser;
			letUpPreset50 = false;
		} else if(!j.getRawButton(preset50Button)) {
			letUpPreset50 = true;
		}
		
		if(j.getRawButton(preset100Button) && letUpPreset100) {
			mult[currentGain] += 1 * inverser;
			letUpPreset100 = false;
		} else if(!j.getRawButton(preset100Button)) {
			letUpPreset100 = true;
		}
	}
}
