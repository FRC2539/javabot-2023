package frc.robot.utils;

import frc.lib.logging.LoggableInteger;

public class AutoPlaceManager {
    private static LoggableInteger rowLogger = new LoggableInteger("/AutoPlace/Row");
    private static LoggableInteger levelLogger = new LoggableInteger("/AutoPlace/Level");

    // Given that we see out of the driver station, it needs to be relative to that view.
    private static int row = 1; // Leftmost row (1-9)
    private static int level = 3; // Hybrid - 3, Mid - 2, High - 1

    private static final int numberOfRows = 9;
    private static final int numberOfLevels = 3;

    public static void initializeAutoPlaceManager() {
        rowLogger.set(row);
        levelLogger.set(level);
    }

    public static void incrementRow() {
        if ((row + 1) > numberOfRows) row = 1;
        else row++;

        rowLogger.set(row);
    }

    public static void decrementRow() {
        if ((row - 1) < 1) row = numberOfRows;
        else row--;

        rowLogger.set(row);
    }

    public static void incrementLevel() {
        if ((level + 1) > numberOfLevels) level = 1;
        else level++;

        levelLogger.set(level);
    }

    public static void decrementLevel() {
        if ((level - 1) < 1) level = numberOfLevels;
        else level--;

        levelLogger.set(level);
    }

    /**
     * @return The row relative to the driver station
     */
    public static int getRow() {
        return row;
    }

    /**
     * @return The level relative to the driver station
     */
    public static int getLevel() {
        return level;
    }

    public static int getRowFieldRelative() {
        return (numberOfRows - 1) - row;
    }

    public static int getLevelFieldRelative() {
        return (numberOfLevels - 1) - level;
    }
}
