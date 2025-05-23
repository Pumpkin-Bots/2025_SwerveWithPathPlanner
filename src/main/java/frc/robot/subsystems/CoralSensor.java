package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import frc.robot.Constants;

/**
 * CoralSensor class is a singleton class that represents the Coral Sensor
 * connected to the robot. It uses the Canandcolor class to communicate with the
 * sensor and get the proximity value. The class provides a method to check if
 * coral is detected by the sensor.
 * 
 * See https://docs.reduxrobotics.com/canandcolor/programming/ for
 * more information on the Canandcolor class.
 */
public class CoralSensor {
    private static CoralSensor instance;
    private final int canId = Constants.Sensors.kCoralSensorId; // CAN ID of the Coral Sensor
    private static final double coralThreshold = 0.15; // Threshold for coral detection

    private Canandcolor canandcolor;

    private CoralSensor() {
        this.canandcolor = new Canandcolor(canId); // Initialize Canandcolor with CAN ID
    }

    // Public method to get the single instance
    public static CoralSensor getInstance() {
        if (instance == null) {
            instance = new CoralSensor();
        }
        return instance;
    }

    public static boolean isCoralDetected() {
        return isConnected() && getInstance().canandcolor.getProximity() < coralThreshold;
    }

    private static boolean isConnected() {
        return getInstance().canandcolor.isConnected();
    }
}
