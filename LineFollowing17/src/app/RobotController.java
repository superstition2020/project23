package app;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

class RobotController implements Runnable {
    // Constants for robot behaviour

    private static final int INTENSITY_THRESHOLD = 15;
    private static final int FORWARD_SPEED = 300;
    private static final int TURN_SPEED = 150;
    private static final int TURN_DURATION = 500;
    private static final int AVOIDANCE_MOVE_DURATION = 1000; 
    private static final float OBSTACLE_DISTANCE_THRESHOLD = 0.1f; // 10 cm

    // Hardware components

    private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
    private SampleProvider colorSampleProvider = colorSensor.getRedMode();
    private EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.C);
    private EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
    private EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
    private SampleProvider distanceSampleProvider = ultrasonicSensor.getDistanceMode();

    // Flag to control the robot thread

    private volatile boolean running = true;
    
    // Start the robot controller thread
    public void start() {
        Thread robotThread = new Thread(this);
        robotThread.start();
    }
    @Override
    public void run() {
        while (running) {
            // Read colour intensity and distance from sensors

            float[] colorSample = new float[colorSampleProvider.sampleSize()];
            colorSampleProvider.fetchSample(colorSample, 0);
            int intensity = (int) (colorSample[0] * 100);

            float[] distanceSample = new float[distanceSampleProvider.sampleSize()];
            distanceSampleProvider.fetchSample(distanceSample, 0);
            float distance = distanceSample[0];

            // Display intensity and distance on the LCD screen
            LCD.clear();
            LCD.drawString("Intensity: " + intensity, 0, 0);
            LCD.drawString("Distance: " + distance, 0, 1);

            // Adjust the robot's movement based on the intensity value and obstacle detection
            if (intensity < INTENSITY_THRESHOLD) {
                // Move forward
                leftMotor.setSpeed(TURN_SPEED);
                rightMotor.setSpeed(100);
                leftMotor.forward();
                rightMotor.forward();
            } else {
                // Turn right
                turnRight();
            }

            // If an obstacle is detected, stop and avoid it
            if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
            	Sound.beep();
                avoidObstacle();
                searchForLine();
                //if (!isLineFound()) {
                  //  searchForLineStraight();
				//	searchForLine();
                //}
            }
        }

        // Stop motors when the thread is stopped
        stopMotors();
    }
    
    // Check if the colour sensor detects a line

	private boolean isLineFound() {
        float[] colorSample = new float[colorSampleProvider.sampleSize()];
        colorSampleProvider.fetchSample(colorSample, 0);
        int intensity = (int) (colorSample[0] * 100);
        return intensity < INTENSITY_THRESHOLD;
    }
    // Search for a line by moving forward until a line is detected


    private void searchForLineStraight() {
        while (!isLineFound()) {
            leftMotor.setSpeed(FORWARD_SPEED);
            rightMotor.setSpeed(100); // Adjusting right motor speed to correct for drift
            leftMotor.forward();
            rightMotor.forward();
        }
        stopMotors();
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(100); // Adjusting right motor speed to correct for drift
        leftMotor.forward();
        rightMotor.forward();
    }
    
    // Turn the robot right

    private void turnRight() {
        rightMotor.setSpeed(TURN_SPEED);
        leftMotor.setSpeed(100); // Adjusting right motor speed to correct for drift
        leftMotor.forward();
        rightMotor.forward();
        //Delay.msDelay(TURN_DURATION); // Allow time for turning
        //stopMotors();
    }
 // Stop both motors

    private void stopMotors() {
        leftMotor.stop(true);
        rightMotor.stop(true);
    }
    
    // Avoid obstacle by turning left, moving forward, and then realigning

    private void avoidObstacle() {
        // Turn left 90 degrees
        leftMotor.setSpeed(TURN_SPEED);
        rightMotor.setSpeed(100); // Adjusting right motor speed to correct for drift
        leftMotor.forward();
        rightMotor.backward();
        Delay.msDelay(TURN_DURATION);
        
        // Move forward a bit to clear the obstacle
        leftMotor.setSpeed(550);
        rightMotor.setSpeed(550);
        leftMotor.forward();
        rightMotor.forward();
        Delay.msDelay(1000);
        
        // Turn right 90 degrees to realign with the line
        leftMotor.setSpeed(100); // Adjusting left motor speed to correct for alignment
        rightMotor.setSpeed(150);
        leftMotor.backward();
        rightMotor.forward();
        Delay.msDelay(1000);
        
        // Continue following the line
        leftMotor.setSpeed(550);
        rightMotor.setSpeed(550);
        leftMotor.forward();
        rightMotor.forward();
        Delay.msDelay(1000);
    }
    // Search for a line by rotating until it is found

    private void searchForLine() {
        // Search for line rotating till found it
        while (true) {
            float[] colorSample = new float[colorSampleProvider.sampleSize()];
            colorSampleProvider.fetchSample(colorSample, 0);
            int intensity = (int) (colorSample[0] * 100);

            if (intensity < INTENSITY_THRESHOLD) {
                break; //found the line
            }

            // rotating slowly to search for the line
            leftMotor.setSpeed(TURN_SPEED);
            rightMotor.setSpeed(100); // Adjusting right motor speed to correct for drift
            leftMotor.forward();
            rightMotor.backward();
        }
        
        // Stop when the line is found
        stopMotors();
        
        // Continue with line following
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(100); // Adjusting right motor speed to correct for drift
        leftMotor.forward();
        rightMotor.forward();
    }
    // Stop the robot controller thread

    public void stop() {
        running = false;
    }
}