
package org.usfirst.frc.team2601.robot;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	CANTalon leftMotor, rightMotor;
	RobotDrive drive;
	Joystick stick;
	JoystickButton enterPID;
	Encoder enc = new Encoder(0,1,true, Encoder.EncodingType.k4X);
	PIDController control;
	
	public double x_val;
	public double y_val;
	public double Kp = 0.65;
	public double Ki = 0.15;
	public double Kd = 0.10;
	public double setpoint = 0.0;
	
    public void robotInit() {
    	leftMotor = new CANTalon(1);
    	rightMotor = new CANTalon(2);
    	drive = new RobotDrive(leftMotor, rightMotor);
    	stick = new Joystick(0);
    	enterPID = new JoystickButton(stick, 1);
    	enc.reset();
    	enc.setDistancePerPulse(.17);
    	enc.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
    	
    	String csvFile = "/pidVals/pid.csv";
    	BufferedReader br = null;
    	String line = "";
    	String cvsSplitBy = ";";
     
    	try {
     
    		br = new BufferedReader(new FileReader(csvFile));
    		while ((line = br.readLine()) != null) {
    			String[] csvVals = line.split(cvsSplitBy);
    			
    			Kp = Double.parseDouble(csvVals[0]);
    			Ki = Double.parseDouble(csvVals[1]);
    			Kd = Double.parseDouble(csvVals[2]);
    			setpoint = Double.parseDouble(csvVals[3]);
    		}
     
    	} catch (FileNotFoundException e) {
    		e.printStackTrace();
    	} catch (IOException e) {
    		e.printStackTrace();
    	} finally {
    		if (br != null) {
    			try {
    				br.close();
    			} catch (IOException e) {
    				e.printStackTrace();
    			}
    		}
    	}
    	
    	control = new PIDController(Kp, Ki, Kd,  enc, leftMotor);
    	control.setSetpoint(setpoint);
    	control.setOutputRange(0.0001, 0.9999);
    	control.startLiveWindowMode();
    	
    	
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
       // x_val = stick.getX();
       // y_val = stick.getY();
        
        drive.arcadeDrive(stick);
        System.out.println(Kd);
        System.out.println(setpoint);
        //control.startLiveWindowMode();
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	    	
    	//control.startLiveWindowMode();
    	control.enable();
    	
    	//enc.startLiveWindowMode();
    	SmartDashboard.putNumber("left moto", leftMotor.get());
    	SmartDashboard.putNumber("encode", enc.getDistance());
    	System.out.println("lm");
    	System.out.println(leftMotor.get());
    	rightMotor.set(leftMotor.get()*-1);
    	//System.out.println(rightMotor.get());
    	System.out.println("enc");
    	System.out.println(enc.getDistance());
    }
    
}
