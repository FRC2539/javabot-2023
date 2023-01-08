package frc.lib.controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;

public class CustomXboxController {
    private final Joystick joystick;

    private final Trigger buttonA;
    private final Trigger buttonB;
    private final Trigger buttonX;
    private final Trigger buttonY;
    private final Trigger leftBumper;
    private final Trigger rightBumper;
    private final Trigger leftRhombus;
    private final Trigger rightRhombus;
    private final Trigger leftThumbButton;
    private final Trigger rightThumbButton;

    private final Axis leftXAxis;
    private final Axis leftYAxis;
    private final Axis leftTrigger;
    private final Axis rightTrigger;
    private final Axis rightXAxis;
    private final Axis rightYAxis;

    private HashMap<String, String> buttonPurposeHashMap = new HashMap<String, String>();

    /**
     * @param port The port the controller is on
     */
    public CustomXboxController(int port) {
        joystick = new Joystick(port);

        buttonA = new JoystickButton(joystick, 1);
        buttonB = new JoystickButton(joystick, 2);
        buttonX = new JoystickButton(joystick, 3);
        buttonY = new JoystickButton(joystick, 4);
        leftBumper = new JoystickButton(joystick, 5);
        rightBumper = new JoystickButton(joystick, 6);
        leftRhombus = new JoystickButton(joystick, 7);
        rightRhombus = new JoystickButton(joystick, 8);
        leftThumbButton = new JoystickButton(joystick, 9);
        rightThumbButton = new JoystickButton(joystick, 10);

        leftXAxis = new JoystickAxis(joystick, 0);
        leftYAxis = new JoystickAxis(joystick, 1);
        leftTrigger = new JoystickAxis(joystick, 2);
        rightTrigger = new JoystickAxis(joystick, 3);
        rightXAxis = new JoystickAxis(joystick, 4);
        rightYAxis = new JoystickAxis(joystick, 5);

        buttonPurposeHashMap.put("type", "CustomXboxController");
    }

    public Trigger getButtonA() {
        return buttonA;
    }

    public Trigger getButtonB() {
        return buttonB;
    }

    public Trigger getButtonX() {
        return buttonX;
    }

    public Trigger getButtonY() {
        return buttonY;
    }

    public Trigger getLeftBumper() {
        return leftBumper;
    }

    public Trigger getRightBumper() {
        return rightBumper;
    }

    public Trigger getLeftRhombus() {
        return leftRhombus;
    }

    public Trigger getRightRhombus() {
        return rightRhombus;
    }

    public Trigger getLeftThumb() {
        return leftThumbButton;
    }

    public Trigger getRightThumb() {
        return rightThumbButton;
    }

    public Axis getLeftXAxis() {
        return leftXAxis;
    }

    public Axis getLeftYAxis() {
        return leftYAxis;
    }

    public Axis getLeftTrigger() {
        return leftTrigger;
    }

    public Axis getRightTrigger() {
        return rightTrigger;
    }

    public Axis getRightXAxis() {
        return rightXAxis;
    }

    public Axis getRightYAxis() {
        return rightYAxis;
    }

    public void setLeftRumble(double rumbleValue) {
        joystick.setRumble(RumbleType.kLeftRumble, rumbleValue);
    }

    public void setRightRumble(double rumbleValue) {
        joystick.setRumble(RumbleType.kRightRumble, rumbleValue);
    }

    public void nameButtonA(String purpose) {
        buttonPurposeHashMap.put("buttonA", purpose);
    }

    public void nameButtonB(String purpose) {
        buttonPurposeHashMap.put("buttonA", purpose);
    }

    public void nameButtonX(String purpose) {
        buttonPurposeHashMap.put("buttonX", purpose);
    }

    public void nameButtonY(String purpose) {
        buttonPurposeHashMap.put("buttonY", purpose);
    }

    public void nameLeftBumper(String purpose) {
        buttonPurposeHashMap.put("leftBumper", purpose);
    }

    public void nameRightBumper(String purpose) {
        buttonPurposeHashMap.put("rightBumper", purpose);
    }

    public void nameLeftRhombus(String purpose) {
        buttonPurposeHashMap.put("leftRhombus", purpose);
    }

    public void nameRightRhombus(String purpose) {
        buttonPurposeHashMap.put("rightRhombus", purpose);
    }

    public void nameLeftThumb(String purpose) {
        buttonPurposeHashMap.put("leftThumbButton", purpose);
    }

    public void nameRightThumb(String purpose) {
        buttonPurposeHashMap.put("rightThumbButton", purpose);
    }

    public void nameLeftXAxis(String purpose) {
        buttonPurposeHashMap.put("leftXAxis", purpose);
    }

    public void nameLeftYAxis(String purpose) {
        buttonPurposeHashMap.put("leftYAxis", purpose);
    }

    public void nameLeftTrigger(String purpose) {
        buttonPurposeHashMap.put("leftTrigger", purpose);
    }

    public void nameRightTrigger(String purpose) {
        buttonPurposeHashMap.put("rightTrigger", purpose);
    }

    public void nameRightXAxis(String purpose) {
        buttonPurposeHashMap.put("rightXAxis", purpose);
    }

    public void nameRightYAxis(String purpose) {
        buttonPurposeHashMap.put("rightYAxis", purpose);
    }
}
