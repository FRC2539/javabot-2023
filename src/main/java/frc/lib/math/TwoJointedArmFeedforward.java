package frc.lib.math;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/*
 * This all was made possible using this whitepaper.
 * https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 *
 * Thank you to Team 449!
 */

public class TwoJointedArmFeedforward {
    Matrix<N2, N2> motorTorqueInvMatrix;

    Matrix<N2, N2> backEmfMatrix;

    double l1, l2;
    double r1, r2;
    double m1, m2;
    double I1, I2;
    double G1, G2;
    double N1, N2;
    double Kt, Kv, R;

    double g;

    /**
     * Initializes a feedforward for voltage calculation of a two jointed arm given the proper input values.
     * Unless otherwise stated, all units are in metric.
     * @param length1
     * Distance between first and second joint.
     * @param length2
     * Distance between second joint and end of arm.
     * @param com1
     * Distance from the first joint to the center of mass of the first arm segment.
     * @param com2
     * Distance from the second joint to the center of mass of the second arm segment.
     * @param mass1
     * Mass of the first arm segment.
     * @param mass2
     * Mass of the second arm segment.
     * @param I1
     * Moment of inertia of the first arm segment.
     * @param I2
     * Moment of inertia of the second arm segment.
     * @param gearRatio1
     * Gear ratio of the motor(s) for the first joint.
     * @param gearRatio2
     * Gear ratio of the motor(s) for the second joint.
     * @param nMotors1
     * Number of motors being used on the first joint.
     * @param nMotors2
     * Number of motors being used on the second joint,
     * @param stallTorque
     * The stall torque of the motors.
     * @param stallCurrent
     * The stall current of the motors.
     * @param freeSpeed
     * The free speed of the motors.
     */
    public TwoJointedArmFeedforward(
            double length1,
            double length2,
            double com1,
            double com2,
            double mass1,
            double mass2,
            double momentOfInertia1,
            double momentOfInertia2,
            double gearRatio1,
            double gearRatio2,
            double nMotors1,
            double nMotors2,
            double stallTorque,
            double stallCurrent,
            double freeSpeed,
            double gravity) {
        l1 = length1;
        l2 = length2;
        r1 = com1;
        r2 = com2;
        m1 = mass1;
        m2 = mass2;
        I1 = momentOfInertia1;
        I2 = momentOfInertia2;
        G1 = gearRatio1;
        G2 = gearRatio2;
        N1 = nMotors1;
        N2 = nMotors2;
        Kt = stallTorque / stallCurrent;
        Kv = freeSpeed / 12;
        R = 12 / stallCurrent;
        g = gravity;

        motorTorqueInvMatrix = calculateMotorTorqueMatrix().inv();
        backEmfMatrix = calculateBackEmfMatrix();
    }

    public Matrix<N2, N2> calculateArmInertiaMatrix(Matrix<N2, N1> motorAngles) {
        GcOrsFunction c = csLambdaCreator(motorAngles, true);

        return new MatBuilder<N2, N2>(Nat.N2(), Nat.N2())
                .fill(
                        m1 * (r1 * r1) + m2 * (l1 * l1 + r2 * r2) + I1 + I2 + 2 * m2 * l1 * r2 * c.a(2),
                        m2 * (r2 * r2) + I2 + m2 * l1 * r2 * c.a(2),
                        m2 * (r2 * r2) + I2 + m2 * l1 * r2 * c.a(2),
                        m2 * (r2 * r2) + I2);
    }

    public Matrix<N2, N2> calculateCoriolisMatrix(Matrix<N2, N1> motorSpeeds, Matrix<N2, N1> motorAngles) {
        GcOrsFunction s = csLambdaCreator(motorAngles, false);

        double dth1 = motorSpeeds.get(0, 0);
        double dth2 = motorSpeeds.get(1, 0);

        return new MatBuilder<N2, N2>(Nat.N2(), Nat.N2())
                .fill(
                        -m2 * l1 * r2 * s.a(2) * dth2,
                        -m2 * l1 * r2 * s.a(2) * (dth1 + dth2),
                        m2 * l1 * r2 * s.a(2) * dth1,
                        0);
    }

    public double[] calculateFeedforwardVoltages(
            double joint1Angle,
            double joint2Angle,
            double joint1Speed,
            double joint2Speed,
            double joint1Acceleration,
            double joint2Acceleration) {
        Matrix<N2, N1> voltages = calculateFeedforwardVoltageMatrix(
                VecBuilder.fill(joint1Acceleration, joint2Acceleration),
                VecBuilder.fill(joint1Speed, joint2Speed),
                VecBuilder.fill(joint1Angle, joint2Angle));
        return new double[] {voltages.get(0, 0), voltages.get(1, 0)};
    }

    public Matrix<N2, N1> calculateFeedforwardVoltageMatrix(
            Matrix<N2, N1> motorAccelerations, Matrix<N2, N1> motorSpeeds, Matrix<N2, N1> motorAngles) {
        return motorTorqueInvMatrix.times(calculateArmInertiaMatrix(motorAngles)
                .times(motorAccelerations)
                .plus(calculateCoriolisMatrix(motorSpeeds, motorAngles).times(motorSpeeds))
                .plus(calculateGravityMatrix(motorAngles))
                .plus(backEmfMatrix.times(motorSpeeds)));
    }

    public Matrix<N2, N1> calculateGravityMatrix(Matrix<N2, N1> motorAngles) {
        GcOrsFunction c = csLambdaCreator(motorAngles, true);

        return new MatBuilder<N2, N1>(Nat.N2(), Nat.N1())
                .fill((m1 * r1 + m2 * l1) * g * c.a(1) + m2 * r2 * g * c.a(1, 2), m2 * r2 * g * c.a(1, 2));
    }

    public Matrix<N2, N2> calculateMotorTorqueMatrix() {
        return new MatBuilder<N2, N2>(Nat.N2(), Nat.N2()).fill(G1 * N1 * Kt / R, 0, 0, G2 * N2 * Kt / R);
    }

    public Matrix<N2, N2> calculateBackEmfMatrix() {
        return new MatBuilder<N2, N2>(Nat.N2(), Nat.N2())
                .fill(G1 * G1 * N1 * Kt / (Kv * R), 0, 0, G2 * G2 * N2 * Kt / (Kv * R));
    }

    // This is beautiful, please dont stop me.
    @FunctionalInterface
    interface GcOrsFunction {
        public Double a(Integer... ints);
    }

    private GcOrsFunction csLambdaCreator(Matrix<N2, N1> motorAngles, boolean isCosine) {
        double theta1 = motorAngles.get(0, 0);
        double theta2 = motorAngles.get(1, 0);
        GcOrsFunction gcorsFunction = (Integer... theta) -> {
            double sum = 0;
            for (int i : theta) sum += i == 1 ? theta1 : theta2;
            return isCosine ? Math.cos(sum) : Math.sin(sum);
        };
        return gcorsFunction;
    }
}
