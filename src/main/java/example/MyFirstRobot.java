package example;

import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import ev3dev.actuators.lego.motors.EV3MediumRegulatedMotor;
import ev3dev.sensors.Battery;
import ev3dev.sensors.ev3.EV3ColorSensor;
import ev3dev.sensors.ev3.EV3TouchSensor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MyFirstRobot {

    private static void Run () {

    }

    public static void main(final String[] args){

        // MOTORS

        System.out.println("EV3-DeliveryTruck: Creating Motor C - Driving");
        final EV3MediumRegulatedMotor motorDrive = new EV3MediumRegulatedMotor(MotorPort.C);
        System.out.println("EV3-DeliveryTruck: Creating Motor D - Steering");
        final EV3MediumRegulatedMotor motorSteer = new EV3MediumRegulatedMotor(MotorPort.D);
        System.out.println("Motor initialized");

        // Sensors

        System.out.println("EV3-DeliveryTruck: Creating Sensor S3 - Linereader");
        final LineReaderV2 lineReader = new LineReaderV2(SensorPort.S3);
        System.out.println("EV3-DeliveryTruck: Creating Sensor S1 - Left Camera");
        final NXTCamV5 leftCam = new NXTCamV5(SensorPort.S1);
        System.out.println("EV3-DeliveryTruck: Creating Sensor S4 - Right Camera");
        final NXTCamV5 rightCam = new NXTCamV5(SensorPort.S4);
        System.out.println("Sensors initialized");

        //To Stop the motor in case of pkill java for example
        Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
            public void run() {
                System.out.println("Emergency Stop");
                motorDrive.stop();
                motorSteer.stop();
            }
        }));

        //=======================================
        /* DRIVING TEST
        //=======================================

        System.out.println("Driving test");
        Delay.msDelay(1000);

        motorDrive.setSpeed(400);
        motorDrive.backward();

        Delay.msDelay(2000);

        motorSteer.setSpeed(300);
        motorSteer.forward();

        Delay.msDelay(2000);

        motorSteer.backward();

        Delay.msDelay(2000);

        motorSteer.stop(true);

        motorDrive.stop(true);

        Delay.msDelay(2000);

        motorDrive.forward();

        Delay.msDelay(2000);

        motorDrive.stop(true);

        */

        //=======================================
        // Linereader Test
        //=======================================

        //lineReader.wake();

        //for (int i = 0; i <= 40; i++){
        //    System.out.println(lineReader.getPIDValue());
        //    Delay.msDelay(1000);
        //}

        //lineReader.sleep();

        //=======================================
        // Camera Test
        //=======================================







        System.out.println("Checking Battery");
        System.out.println("Votage: " + Battery.getInstance().getVoltage());

        System.exit(0);
    }
}
