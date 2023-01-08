package frc.lib.controller;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class ThrustmasterJoystick {
    private int port;

    private final Joystick joystick;

    private final Trigger trigger;
    private final Trigger bottomThumb;
    private final Trigger leftThumb;
    private final Trigger rightThumb;
    private final Trigger leftTopLeft;
    private final Trigger leftTopMiddle;
    private final Trigger leftTopRight;
    private final Trigger leftBottomRight;
    private final Trigger leftBottomMiddle;
    private final Trigger leftBottomLeft;
    private final Trigger rightTopRight;
    private final Trigger rightTopMiddle;
    private final Trigger rightTopLeft;
    private final Trigger rightBottomLeft;
    private final Trigger rightBottomMiddle;
    private final Trigger rightBottomRight;

    private final Axis xAxis;
    private final Axis yAxis;
    private final Axis zAxis;
    private final Axis sliderAxis;

    private HashMap<String, String> buttonPurposeHashMap = new HashMap<String, String>();

    /**
     * @param port The port the controller is on
     */
    public ThrustmasterJoystick(int port) {
        this.port = port;

        joystick = new Joystick(port);

        trigger = new JoystickButton(joystick, 1);
        bottomThumb = new JoystickButton(joystick, 2);
        leftThumb = new JoystickButton(joystick, 3);
        rightThumb = new JoystickButton(joystick, 4);
        leftTopLeft = new JoystickButton(joystick, 5);
        leftTopMiddle = new JoystickButton(joystick, 6);
        leftTopRight = new JoystickButton(joystick, 7);
        leftBottomRight = new JoystickButton(joystick, 8);
        leftBottomMiddle = new JoystickButton(joystick, 9);
        leftBottomLeft = new JoystickButton(joystick, 10);
        rightTopRight = new JoystickButton(joystick, 11);
        rightTopMiddle = new JoystickButton(joystick, 12);
        rightTopLeft = new JoystickButton(joystick, 13);
        rightBottomLeft = new JoystickButton(joystick, 14);
        rightBottomMiddle = new JoystickButton(joystick, 15);
        rightBottomRight = new JoystickButton(joystick, 16);

        xAxis = new JoystickAxis(joystick, 0);
        yAxis = new JoystickAxis(joystick, 1);
        zAxis = new JoystickAxis(joystick, 2);
        sliderAxis = new JoystickAxis(joystick, 3);
        sliderAxis.setInverted(true);

        buttonPurposeHashMap.put("type", "ThrustmasterJoystick");
    }

    public Trigger getTrigger() {
        return trigger;
    }

    public Trigger getBottomThumb() {
        return bottomThumb;
    }

    public Trigger getLeftThumb() {
        return leftThumb;
    }

    public Trigger getRightThumb() {
        return rightThumb;
    }

    public Trigger getLeftTopLeft() {
        return leftTopLeft;
    }

    public Trigger getLeftTopMiddle() {
        return leftTopMiddle;
    }

    public Trigger getLeftTopRight() {
        return leftTopRight;
    }

    public Trigger getLeftBottomRight() {
        return leftBottomRight;
    }

    public Trigger getLeftBottomMiddle() {
        return leftBottomMiddle;
    }

    public Trigger getLeftBottomLeft() {
        return leftBottomLeft;
    }

    public Trigger getRightTopLeft() {
        return rightTopLeft;
    }

    public Trigger getRightTopMiddle() {
        return rightTopMiddle;
    }

    public Trigger getRightTopRight() {
        return rightTopRight;
    }

    public Trigger getRightBottomRight() {
        return rightBottomRight;
    }

    public Trigger getRightBottomMiddle() {
        return rightBottomMiddle;
    }

    public Trigger getRightBottomLeft() {
        return rightBottomLeft;
    }

    public Axis getXAxis() {
        return xAxis;
    }

    public Axis getYAxis() {
        return yAxis;
    }

    public Axis getZAxis() {
        return zAxis;
    }

    public Axis getSliderAxis() {
        return sliderAxis;
    }

    public void nameTrigger(String purpose) {
        buttonPurposeHashMap.put("trigger", purpose);
    }

    public void nameBottomThumb(String purpose) {
        buttonPurposeHashMap.put("bottomThumb", purpose);
    }

    public void nameLeftThumb(String purpose) {
        buttonPurposeHashMap.put("leftThumb", purpose);
    }

    public void nameRightThumb(String purpose) {
        buttonPurposeHashMap.put("rightThumb", purpose);
    }

    public void nameLeftTopLeft(String purpose) {
        buttonPurposeHashMap.put("leftTopLeft", purpose);
    }

    public void nameLeftTopMiddle(String purpose) {
        buttonPurposeHashMap.put("leftTopMiddle", purpose);
    }

    public void nameLeftTopRight(String purpose) {
        buttonPurposeHashMap.put("leftTopRight", purpose);
    }

    public void nameLeftBottomRight(String purpose) {
        buttonPurposeHashMap.put("leftBottomRight", purpose);
    }

    public void nameLeftBottomMiddle(String purpose) {
        buttonPurposeHashMap.put("leftBottomMiddle", purpose);
    }

    public void nameLeftBottomLeft(String purpose) {
        buttonPurposeHashMap.put("leftBottomLeft", purpose);
    }

    public void nameRightTopLeft(String purpose) {
        buttonPurposeHashMap.put("rightTopLeft", purpose);
    }

    public void nameRightTopMiddle(String purpose) {
        buttonPurposeHashMap.put("rightTopMiddle", purpose);
    }

    public void nameRightTopRight(String purpose) {
        buttonPurposeHashMap.put("rightTopRight", purpose);
    }

    public void nameRightBottomRight(String purpose) {
        buttonPurposeHashMap.put("rightBottomRight", purpose);
    }

    public void nameRightBottomMiddle(String purpose) {
        buttonPurposeHashMap.put("rightBottomMiddle", purpose);
    }

    public void nameRightBottomLeft(String purpose) {
        buttonPurposeHashMap.put("rightBottomLeft", purpose);
    }

    public void nameXAxis(String purpose) {
        buttonPurposeHashMap.put("xAxis", purpose);
    }

    public void nameYAxis(String purpose) {
        buttonPurposeHashMap.put("yAxis", purpose);
    }

    public void nameZAxis(String purpose) {
        buttonPurposeHashMap.put("zAxis", purpose);
    }

    public void nameSliderAxis(String purpose) {
        buttonPurposeHashMap.put("sliderAxis", purpose);
    }

    public void sendButtonNamesToNT() {
        NetworkTableInstance.getDefault()
                .getTable("Controllers")
                .getEntry(port + "")
                .setString(toJSON());
    }

    /**
     * @return Button names as a JSON String
     */
    public String toJSON() {
        return buttonPurposeHashMap.entrySet().stream()
                .map((Map.Entry<String, String> buttonEntry) -> stringifyButtonName(buttonEntry))
                .collect(Collectors.joining(", ", "{", "}"));
    }

    private String stringifyButtonName(Map.Entry<String, String> buttonEntry) {
        return "\"" + buttonEntry.getKey() + "\": " + "\"" + buttonEntry.getValue() + "\"";
    }
}
