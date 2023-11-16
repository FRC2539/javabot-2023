package frc.lib.math;

public class Conversions {

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Radians of Rotation of Mechanism
     */
    public static double falconToRadians(double counts, double gearRatio) {
        return falconToDegrees(counts, gearRatio) / 180 * Math.PI;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param radians Radians of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double radiansToFalcon(double radians, double gearRatio) {
        return (degreesToFalcon(radians / Math.PI * 180, gearRatio));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RadPS)
     * @return Radians Per Second of Mechanism
     */
    public static double falconToRadPS(double velocityCounts, double gearRatio) {
        return falconToRPM(velocityCounts, gearRatio) * 2 * Math.PI / 60;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param RPM Radians per Second of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RadPS)
     * @return Falcon Units of Mechanism
     */
    public static double RadPStoFalcon(double RadPS, double gearRatio) {
        return RPMToFalcon(RadPS * 60 / (2 * Math.PI), gearRatio);
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param rotations Rotations from the Motor
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Radians of Rotation of Mechanism
     */
    public static double motorToRadians(double rotations, double gearRatio) {
        return rotations * (Math.PI * 2 / gearRatio);
    }

    /**
     * @param radians Radians of Rotation of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Rotations of the Motor
     */
    public static double radiansToMotor(double radians, double gearRatio) {
        return radians / (Math.PI * 2) * gearRatio;
    }

    /**
     * @param rotations Rotations from the Motor
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param circumference Circumference of the Mechanism
     * @return Meters of Movement of Mechanism
     */
    public static double motorToMeters(double rotations, double circumference, double gearRatio) {
        return rotations * circumference / gearRatio;
    }

    /**
     * @param meters Meters of Movement of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param circumference Circumference of the Mechanism
     * @return Rotations Per Second of Motor
     */
    public static double MetersToMotor(double meters, double circumference, double gearRatio) {
        return meters * gearRatio / circumference;
    }

    /**
     * @param RPS Rotations per Second for the Motor
     * @param gearRatioGear Ratio between Motor and Mechanism
     * @return Radians per Second of Rotation of Mechanism
     */
    public static double motorRPSToRadPS(double RPS, double gearRatio) {
        return RPS * (Math.PI * 2) / gearRatio;
    }

    /**
     * @param radians Radians Per Second of Rotation of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Rotations Per Second of the Motor
     */
    public static double RadPSToMotorRPS(double RadPS, double gearRatio) {
        return RadPS / (Math.PI * 2) * gearRatio;
    }

    /**
     * @param RPS Rotations per Second for the Motor
     * @param gearRatio Gear Ratio between Motor and Mechanism\
     * @param circumference Circumference of the Mechanism
     * @return Meters per Second of Movement of Mechanism
     */
    public static double motorRPSToMPS(double RPS, double circumference, double gearRatio) {
        return RPS / gearRatio * circumference;
    }

    /**
     * @param MPS Meters Per Second of Movement of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param circumference Circumference of the Mechanism
     * @return Rotations Per Second of Motor
     */
    public static double MPSToMotorRPS(double MPS, double circumference, double gearRatio) {
        return MPS * gearRatio / circumference;
    }
}
