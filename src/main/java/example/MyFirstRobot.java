package example;

import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import ev3dev.actuators.lego.motors.EV3MediumRegulatedMotor;
import ev3dev.sensors.Battery;
import ev3dev.sensors.ev3.EV3ColorSensor;
import ev3dev.sensors.ev3.EV3IRSensor;
import ev3dev.sensors.ev3.EV3TouchSensor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MyFirstRobot {

    private static int LineReader() {
        //TODO: PLACE YOUR CODE HERE

        return 0;
    }

    private static int steerAngle() {
        //TODO: PLACE YOUR CODE HERE

        return 0;
    }

    private static int move() {
        //TODO: PLACE YOUR CODE HERE

        return 0;
    }


    private static void DTruckRun () {

        move();
        //TODO: PLACE YOUR CODE HERE

    }

    //motor for drive forwards and backwards - connected to motor port D
    public static EV3MediumRegulatedMotor motorDrive;
    //motor for steering - connected to motor port C
    public static EV3MediumRegulatedMotor motorSteer;

    //motor for grabber - connected to motor port A
    //public static EV3MediumRegulatedMotor craneGrabber;

    //sensor for proximity - connect to sensor port S1
    public static EV3IRSensor sensorProximity;
    //sensor for line reading - connected to sensor port S4
    public static LineReaderV2 lineReader;

    public static void main(final String[] args){

        double minVoltage = 7.200;

        //Always check if battery voltage is enougth
        System.out.println("Battery Voltage: " + Battery.getInstance().getVoltage());
        System.out.println("Battery Current: " + Battery.getInstance().getBatteryCurrent());
        if (Battery.getInstance().getVoltage() < minVoltage) {
            System.out.println("Battery voltage to low. Shutdown down and change the batteries.");
            System.exit(0);
        }

        // initialize all motors here
        System.out.println("EV3-DTruck: Creating Motor C - Driving");
        motorDrive = new EV3MediumRegulatedMotor(MotorPort.C);
        System.out.println("EV3-DTruck: Creating Motor D - Steering");
        motorSteer = new EV3MediumRegulatedMotor(MotorPort.D);

        // initialize all sensors here
        lineReader = new LineReaderV2(SensorPort.S4);
        sensorProximity = new EV3IRSensor(SensorPort.S1);
        System.out.println("Sensors initialized");

        //To Stop the motor in case of pkill java for example
        Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
            public void run() {
                System.out.println("Emergency Stop");
                motorDrive.stop();
                motorSteer.stop();
            }
        }));

        System.out.println("EV3-Forklift: Defining the Stop mode");
        motorDrive.brake();
        motorSteer.brake();





        //Main class for executing code
        DTruckRun();





        System.out.println("Checking Battery");
        System.out.println("Votage: " + Battery.getInstance().getVoltage());
    }
}
