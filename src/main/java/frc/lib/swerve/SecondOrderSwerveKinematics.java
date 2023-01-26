package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;
import java.util.Collections;
import org.ejml.simple.SimpleMatrix;

/*
Dear Team 95,

Thank you for making your second order inverse kinematics and
forward kinematics code public. It is pure witchcraft and I
would cry if I had to write it.

Sincerely, Matthew

Link to original whitepaper by Team 449:
https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
*/

/**
 * Clone of WPI SwerveKinematics, which implements second order
 * kinematics when calculating modules states from chassis speed.
 * <p></p>
 *
 * Makes use of {@link SecondOrderSwerveModuleState} to add the angular
 * velocity that is required of the module as an output.
 */
public class SecondOrderSwerveKinematics extends SwerveDriveKinematics {
    private final SimpleMatrix inverseKinematics;
    private final SimpleMatrix forwardKinematics;
    private final SimpleMatrix accelerationInverseKinematics;

    private final int numModules;
    private final Translation2d[] modules;
    private Translation2d previousCenterOfRotation = new Translation2d();

    /**
     * Constructs a swerve drive kinematics object. This takes in a variable number of wheel locations
     * as Translation2ds. The order in which you pass in the wheel locations is the same order that
     * you will receive the module states when performing inverse kinematics. It is also expected that
     * you pass in the module states in the same order when calling the forward kinematics methods.
     *
     * @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
     */
    public SecondOrderSwerveKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters);
        if (wheelsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        numModules = wheelsMeters.length;
        modules = Arrays.copyOf(wheelsMeters, numModules);
        inverseKinematics = new SimpleMatrix(numModules * 2, 3);
        accelerationInverseKinematics = new SimpleMatrix(numModules * 2, 4);

        for (int i = 0; i < numModules; i++) {
            inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -modules[i].getY());
            inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +modules[i].getX());
            accelerationInverseKinematics.setRow(
                    i * 2 + 0, 0, /* Start Data */ 1, 0, -modules[i].getX(), -modules[i].getY());
            accelerationInverseKinematics.setRow(
                    i * 2 + 1, 0, /* Start Data */ 0, 1, -modules[i].getY(), +modules[i].getX());
        }
        forwardKinematics = inverseKinematics.pseudoInverse();
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. This
     * method is often used to convert joystick values into module speeds and angles.
     *
     * <p>This function also supports variable centers of rotation. During normal operations, the
     * center of rotation is usually the same as the physical center of the robot; therefore, the
     * argument is defaulted to that use case. However, if you wish to change the center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary),
     * the previously calculated module angle will be maintained.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
     *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
     *     component, the robot will rotate around that corner.
     * @return An array containing the module states. Use caution because these module states are not
     *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
     *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SecondOrderSwerveModuleState[], double)
     *     DesaturateWheelSpeeds} function to rectify this issue.
     */
    @SuppressWarnings("PMD.MethodReturnsInternalArray")
    public SecondOrderSwerveModuleState[] toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        SecondOrderSwerveModuleState[] moduleStates = new SecondOrderSwerveModuleState[numModules];
        Arrays.fill(moduleStates, new SecondOrderSwerveModuleState());
        if (chassisSpeeds.vxMetersPerSecond == 0.0
                && chassisSpeeds.vyMetersPerSecond == 0.0
                && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
            for (int i = 0; i < numModules; i++) {
                moduleStates[i].speedMetersPerSecond = 0.0;
            }

            return moduleStates;
        }

        if (!centerOfRotationMeters.equals(previousCenterOfRotation)) {
            for (int i = 0; i < numModules; i++) {
                inverseKinematics.setRow(
                        i * 2 + 0, 0, /* Start Data */ 1, 0, -modules[i].getY() + centerOfRotationMeters.getY());
                inverseKinematics.setRow(
                        i * 2 + 1, 0, /* Start Data */ 0, 1, +modules[i].getX() - centerOfRotationMeters.getX());
                accelerationInverseKinematics.setRow(
                        i * 2 + 0,
                        0, /* Start Data */
                        1,
                        0,
                        -modules[i].getX() + centerOfRotationMeters.getX(),
                        -modules[i].getY() + centerOfRotationMeters.getY());
                accelerationInverseKinematics.setRow(
                        i * 2 + 1,
                        0, /* Start Data */
                        0,
                        1,
                        -modules[i].getY() + centerOfRotationMeters.getY(),
                        +modules[i].getX() - centerOfRotationMeters.getX());
            }
            previousCenterOfRotation = centerOfRotationMeters;
        }

        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);

        SimpleMatrix moduleVelocityStatesMatrix = inverseKinematics.mult(chassisSpeedsVector);

        SimpleMatrix accelerationVector = new SimpleMatrix(4, 1);
        accelerationVector.setColumn(0, 0, 0, 0, Math.pow(chassisSpeeds.omegaRadiansPerSecond, 2), 0);

        SimpleMatrix moduleAccelerationStatesMatrix = accelerationInverseKinematics.mult(accelerationVector);

        for (int i = 0; i < numModules; i++) {
            double x = moduleVelocityStatesMatrix.get(i * 2, 0);
            double y = moduleVelocityStatesMatrix.get(i * 2 + 1, 0);

            double ax = moduleAccelerationStatesMatrix.get(i * 2, 0);
            double ay = moduleAccelerationStatesMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);

            SimpleMatrix trigThetaAngle = new SimpleMatrix(2, 2);
            trigThetaAngle.setColumn(0, 0, angle.getCos(), -angle.getSin());
            trigThetaAngle.setColumn(1, 0, angle.getSin(), angle.getCos());

            SimpleMatrix accelVector = new SimpleMatrix(2, 1);
            accelVector.setColumn(0, 0, ax, ay);

            SimpleMatrix omegaVector = trigThetaAngle.mult(accelVector);

            double omega = omegaVector.get(1, 0) / speed;
            moduleStates[i] = new SecondOrderSwerveModuleState(speed, angle, omega);
        }

        return moduleStates;
    }

    /**
     * Performs inverse kinematics. See {@link #toSwerveModuleStates(ChassisSpeeds, Translation2d)}
     * toSwerveModuleStates for more information.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @return An array containing the module states.
     */
    public SecondOrderSwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the given module states.
     * This method is often used for odometry -- determining the robot's position on the field using
     * data from the real-world speed and angle of each module on the robot.
     *
     * @param wheelStates The state of the modules (as a SwerveModuleState type) as measured from
     *     respective encoders and gyros. The order of the swerve module states should be same as
     *     passed into the constructor of this class.
     * @return The resulting chassis speed.
     */
    public ChassisSpeeds toChassisSpeeds(SecondOrderSwerveModuleState... wheelStates) {
        if (wheelStates.length != numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in " + "constructor");
        }
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(numModules * 2, 1);

        for (int i = 0; i < numModules; i++) {
            SecondOrderSwerveModuleState module = wheelStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
        }

        SimpleMatrix chassisSpeedsVector = forwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0), chassisSpeedsVector.get(1, 0), chassisSpeedsVector.get(2, 0));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(
            SecondOrderSwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond =
                        moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum, as well
     * as getting rid of joystick saturation at edges of joystick.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param currentChassisSpeed The current speed of the robot
     * @param attainableMaxModuleSpeedMetersPerSecond The absolute max speed that a module can reach
     * @param attainableMaxTranslationalSpeedMetersPerSecond The absolute max speed that your robot
     *     can reach while translating
     * @param attainableMaxRotationalVelocityRadiansPerSecond The absolute max speed the robot can
     *     reach while rotating
     */
    public static void desaturateWheelSpeeds(
            SecondOrderSwerveModuleState[] moduleStates,
            ChassisSpeeds currentChassisSpeed,
            double attainableMaxModuleSpeedMetersPerSecond,
            double attainableMaxTranslationalSpeedMetersPerSecond,
            double attainableMaxRotationalVelocityRadiansPerSecond) {
        double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;

        if (attainableMaxTranslationalSpeedMetersPerSecond == 0
                || attainableMaxRotationalVelocityRadiansPerSecond == 0
                || realMaxSpeed == 0) {
            return;
        }
        double translationalK = Math.hypot(currentChassisSpeed.vxMetersPerSecond, currentChassisSpeed.vyMetersPerSecond)
                / attainableMaxTranslationalSpeedMetersPerSecond;
        double rotationalK =
                Math.abs(currentChassisSpeed.omegaRadiansPerSecond) / attainableMaxRotationalVelocityRadiansPerSecond;
        double k = Math.max(translationalK, rotationalK);
        double scale = Math.min(k * attainableMaxModuleSpeedMetersPerSecond / realMaxSpeed, 1);
        for (SwerveModuleState moduleState : moduleStates) {
            moduleState.speedMetersPerSecond *= scale;
        }
    }
}
