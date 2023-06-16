package frc.lib.math;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

/*
 * This all was made possible using this whitepaper.
 * https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 *
 * Thank you to Team 449!
 */

public class AdaptedTwoJointedArmFeedforward {

    double l1, l2;
    double r1, r2;
    double m1, m2;
    double I1, I2;
    double G1, G2;
    double N1, N2;
    double Kt, Kv, R;

    double g;

    DCMotor motor1, motor2;

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
         * Moment of inertia of the second arm segment.
         * @param g
         * The acceleration due to gravity.
         * @param motor1constants
         * The DCMotor representation of the first driving motor with the reduction applied.
         * @param motor2constants
         * The DCMotor representation of the second driving motor with the reduction applied.
     */
    public AdaptedTwoJointedArmFeedforward(
            double length1,
            double length2,
            double com1,
            double com2,
            double mass1,
            double mass2,
            double gravity,
            DCMotor motor1constants,
            DCMotor motor2constants) {
        l1 = length1;
        l2 = length2;
        r1 = com1;
        r2 = com2;
        m1 = mass1;
        m2 = mass2;
        g = gravity;

        motor1 = motor1constants;
        motor2 = motor2constants;
    }

    public Matrix<N2, N1> calculateFeedforwardVoltages(
            double joint1Angle,
            double joint2Angle) {
        
        Matrix<N2, N1> armTorques = calculateNaturalArmTorques(joint1Angle, joint2Angle);

        double motor1Voltage = motor1.getVoltage(-armTorques.get(0, 0), 0);
        double motor2Voltage = motor2.getVoltage(-armTorques.get(1, 0), 0);

        return VecBuilder.fill(motor1Voltage, motor2Voltage);
    }

    /**
     * 
     * 
     * 
     * @param joint1Angle the angle of the mast relative to the floor
     * @param joint2Angle the angle of the boom relative to the mast
     * @return
     */
    public Matrix<N2, N1> calculateNaturalArmTorques(double joint1Angle,
    double joint2Angle) {

        //these formulas are calculated using the fact that Torque = - change in energy / change in rotation
        //aka T = -dPE / dOmega
        double joint1Torque = -g * (r1 * m1 + l1 * m2) * Math.cos(joint1Angle);
        double joint2Torque = -g * r2 * m2 * Math.cos(joint2Angle + joint1Angle);

        return VecBuilder.fill(
            joint1Torque,
            joint2Torque);
    }

    public Matrix<N2, N1> calculateMotorTorquesFromVoltages(double joint1Voltage,
    double joint2Voltage) {

        //these formulas are calculated using the fact that Torque = - change in energy / change in rotation
        //aka T = -dPE / dOmega
        double motor1Torque = motor1.getTorque(joint1Voltage);
        double motor2Torque = motor2.getTorque(joint2Voltage);

        return VecBuilder.fill(
            motor1Torque,
            motor2Torque);
    }
}
