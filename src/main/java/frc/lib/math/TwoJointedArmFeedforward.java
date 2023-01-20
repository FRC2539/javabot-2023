package frc.lib.math;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

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
     *
     * @param length1
     * @param length2
     * @param com1
     * @param com2
     * @param mass1
     * @param mass2
     * @param I1
     * @param I2
     * @param gearRatio1
     * @param gearRatio2
     * @param nMotors1
     * @param nMotors2
     * @param stallTorque
     * @param stallCurrent
     * @param freeSpeed
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
            double joint1Acceleration,
            double joint2Acceleration,
            double joint1Speed,
            double joint2Speed,
            double joint1Angle,
            double joint2Angle) {
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
