package org.usfirst.frc.team6688.robot;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.VideoSource;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import org.usfirst.frc.team6688.robot.GripPipeline;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

public class Robot extends IterativeRobot {
	
    /*CameraServer server;
    VideoSource cam0;
    NetworkTable table;
    private static final int IMG_WIDTH = 640;
    private static final int IMG_HEIGHT = 480;
    private VisionThread visionThread;
    private double centerX=0.0;
    private final Object imgLock = new Object();
    
    public Robot() {
    	table = NetworkTable.getTable("GRIP/myContoursReport");
    }

	public GripPipeline pipeline = new GripPipeline(); */
	
	public static SpeedController driveTrainfrontRight;
    public static SpeedController driveTrainfrontLeft;
    public static SpeedController driveTrainrearRight;
    public static SpeedController driveTrainrearLeft;
    public static RobotDrive driveTrainRobotDrive41;
    
    public static SpeedController climber;
    public static SpeedController motor; 
    
    public static Joystick xBox360;
    public static Joystick SciFor;
    public static Joystick SciBac;
    
    CheesyDriveHelper mCheesyDrive = new CheesyDriveHelper();
    
	final String defaultAuto = "Default";
	final String driveAuto = "Drive Auto";
	final String gearAuto = "Gear Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
    Compressor c = new Compressor();
    DoubleSolenoid Sol = new DoubleSolenoid (1,2);


	@Override
	public void robotInit() {
		
		/*UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT); 
		
		visionThread = new VisionThread(camera, new GripPipeline(), pipepline -> {
			if(!pipeline.filterContoursOutput().isEmpty()){
				Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
				synchronized (imgLock) {
					centerX = r.x + (r.width/2);
				}
			}
		});
		visionThread.start();*/
			

		SmartDashboard.putData(Scheduler.getInstance());
		chooser.addDefault("Do Nothing Auto", defaultAuto);
		chooser.addObject("Drive Forward", driveAuto);
		chooser.addObject("Gear", gearAuto);
		/*SmartDashboard.getNumber("current", c.getCompressorCurrent());
		SmartDashboard.putBoolean("enabled", c.enabled());
		SmartDashboard.putBoolean("pressure", c.getPressureSwitchValue());
		SmartDashboard.putData("Auto choices", chooser);*/
		SmartDashboard.putBoolean("For", Sol.isFwdSolenoidBlackListed());
		SmartDashboard.putBoolean("Rev", Sol.isRevSolenoidBlackListed());
		
		driveTrainfrontRight = new Spark(1);
	    driveTrainfrontLeft = new Spark(0);
	    driveTrainrearRight = new Spark(2);
	    driveTrainrearLeft = new Spark(3);
	    
	    climber = new Spark(4);
	    motor = new Spark (5);
	    
	    xBox360 = new Joystick(0);
	    SciFor = new Joystick (2);
	    SciBac = new Joystick (3);
	    
	    driveTrainRobotDrive41 = new RobotDrive(driveTrainfrontLeft, driveTrainrearLeft, driveTrainfrontRight, driveTrainrearRight);    	    
	    
	    driveTrainRobotDrive41.setSafetyEnabled(false);
	    driveTrainRobotDrive41.setExpiration(0.1);
	    driveTrainRobotDrive41.setSensitivity(0.5);
	    driveTrainRobotDrive41.setMaxOutput(0.5);
	    
	    c.setClosedLoopControl(false);
	    
	    /*boolean enabled = c.enabled();
	    boolean pressureSwitch = c.getPressureSwitchValue();
	    double current = c.getCompressorCurrent();*/
	    
		/*double[] defaultValue = new double[0]; //in the case that the output being read isn't avaiable yet, a deafult array is supplied. 
		while (true){
			double[] areas = table.getNumberArray("area", defaultValue);
			System.out.print("areas: ");
			for (double area: areas) {
				System.out.print(area + " ");
			}
			System.out.println();
			Timer.delay(1);
		}*/
	    
	}
	
	public double startTime;
	
	
	@Override
	public void autonomousInit() {
		/*autoSelected = chooser.getSelected();
		startTime =Timer.getFPGATimestamp();
		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);*/
	}

	
	@Override
	public void autonomousPeriodic() {

			for(int i=0; i<5; i++)
			{
	    	Sol.set(DoubleSolenoid.Value.kForward);
	    	Timer.delay(5.0);
	    	Sol.set(DoubleSolenoid.Value.kReverse);
	    	Timer.delay(5.0);
			}
	    
	    /*if (Timer.getFPGATimestamp() - startTime < 2) {
			Sol.set(DoubleSolenoid.Value.kForward);
	    }
	    else
	    	Sol.set(DoubleSolenoid.Value.kReverse);
	   	    
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		double turn = centerX - (IMG_WIDTH/2);
		/*driveTrainRobotDrive41.arcadeDrive(-0.6, turn * 0.005); //(forward/backwards, rotates) 
		Scheduler.getInstance().run();
		switch (autoSelected) {S\                                                                                 
		case driveAuto:
			if (Timer.getFPGATimestamp() - startTime < 2) {
				driveTrainRobotDrive41.setLeftRightMotorOutputs(1, -1);
				driveTrainfrontLeft.set(1);
		f	driveTrainrearLeft.set(1);
				driveTrainfrontRight.set(-1);
				driveTrainrearRight.set(-1);
			}
			
			else {
				driveTrainfrontLeft.set(0);
				driveTrainrearLeft.set(0);
				driveTrainfrontRight.set(0);
				driveTrainrearRight.set(0);
			}
			break;
		
		case gearAuto: //114 inches
			if(Timer.getFPGATimestamp() - startTime < 0.4){
				driveTrainfrontLeft.set(0.7);
				driveTrainrearLeft.set(0.7);
				driveTrainfrontRight.set(-0.7);
				driveTrainrearRight.set(-0.7);				
			}
			else if(Timer.getFPGATimestamp() - startTime > 0.4 && Timer.getFPGATimestamp() - startTime < 0.7){
				driveTrainfrontLeft.set(0.6);
				driveTrainrearLeft.set(0.6);
				driveTrainfrontRight.set(-0.6);
				driveTrainrearRight.set(-0.6);	
			}
			else if(Timer.getFPGATimestamp() - startTime > 0.7 && Timer.getFPGATimestamp() - startTime < 0.8){
				driveTrainfrontLeft.set(0.5);
				driveTrainrearLeft.set(0.5);
				driveTrainfrontRight.set(-0.5);
				driveTrainrearRight.set(-0.5);					
			}
			else if(Timer.getFPGATimestamp() - startTime > 0.8 && Timer.getFPGATimestamp() - startTime < 0.9){ //2 secs (now, one)
				driveTrainfrontLeft.set(0.4);
				driveTrainrearLeft.set(0.4);
				driveTrainfrontRight.set(-0.4);
				driveTrainrearRight.set(-0.4);			
			}
			else if(Timer.getFPGATimestamp() - startTime > 0.9 && Timer.getFPGATimestamp() - startTime < 1.1){ //2 secs (now, one)
				driveTrainfrontLeft.set(0.3);
				driveTrainrearLeft.set(0.3);
				driveTrainfrontRight.set(-0.3);
				driveTrainrearRight.set(-0.3);					
			}
			else if(Timer.getFPGATimestamp() - startTime > 1.1 && Timer.getFPGATimestamp() - startTime < 1.9){ //8 secs
				driveTrainfrontLeft.set(0.2);
				driveTrainrearLeft.set(0.2);
				driveTrainfrontRight.set(-0.2);
				driveTrainrearRight.set(-0.2);					
			}
			else if(Timer.getFPGATimestamp() - startTime > 1.9 && Timer.getFPGATimestamp() - startTime < 2.2){ //3 secs
				driveTrainfrontLeft.set(0.1);
				driveTrainrearLeft.set(0.1);
				driveTrainfrontRight.set(-0.1);
				driveTrainrearRight.set(-0.1);			
			}
			else{
				driveTrainfrontLeft.set(0);
				driveTrainrearLeft.set(0);
				driveTrainfrontRight.set(0);
				driveTrainrearRight.set(0);			
			}
			break;
			
		case defaultAuto:
		default:
			driveTrainfrontLeft.set(0);
			driveTrainrearLeft.set(0);
			driveTrainfrontRight.set(0);
			driveTrainrearRight.set(0);
			break;
		}*/
	}


	@Override
	public void teleopPeriodic() {
		/*DriveSignal output = mCheesyDrive.cheesyDrive(xBox360.getRawAxis(1), -xBox360.getRawAxis(4), xBox360.getRawButton(5));
		driveTrainfrontLeft.set(-output.leftMotor);
		driveTrainrearLeft.set(-output.leftMotor);
		driveTrainfrontRight.set(output.rightMotor);
		driveTrainrearRight.set(output.rightMotor);	*/
		
		/*double x = xBox360.getRawAxis(1) * .9;
		double y = -xBox360.getRawAxis(0);
		driveTrainRobotDrive41.arcadeDrive(Math.signum(x) * x * x, Math.signum(y) * y * y, false);*/
		//c.setClosedLoopControl(true);
		driveTrainRobotDrive41.arcadeDrive(xBox360);
		
		if (xBox360.getRawButton(6)){
			Sol.set(DoubleSolenoid.Value.kForward);
			}
		else if (xBox360.getRawButton(5)){
			Sol.set(DoubleSolenoid.Value.kReverse);
		}
		
		if (xBox360.getRawButton(4)){
			motor.set(.5);
		}
		else if(xBox360.getRawButton(3)){
			motor.set(-0.5);
		}
		
		else
		{
			motor.set(0);
			Sol.set(DoubleSolenoid.Value.kOff);
		}
		
		
		/*if (xBox360.getRawButton(6)){
			//climber.set(1);
			 Sol.set(DoubleSolenoid.Value.kForward);
		} 
		else {
			//climber.set(0);
			Sol.set(DoubleSolenoid.Value.kReverse);

		}*/
	}


	@Override
	public void testPeriodic() {
	}
}