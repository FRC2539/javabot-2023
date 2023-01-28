package frc.lib.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.logging.LogValue.LoggableType;
import frc.lib.logging.Logger.LogTable;
import frc.lib.logging.LoggingThread.Writer;
import java.util.HashMap;
import java.util.Map;

public class DataLogWriter implements Writer {
    private final DataLog log = DataLogManager.getLog();

    private final Map<String, Integer> entryIDs = new HashMap<>();

    public void sendUpdates(LogTable logTable) {
        for (Map.Entry<String, LogValue> field : logTable.updatesMap().entrySet()) {
            // Create entry if necessary
            LoggableType type = field.getValue().type;
            if (!entryIDs.containsKey(field.getKey())) {
                entryIDs.put(field.getKey(), log.start(field.getKey(), type.getWPILOGType()));
            }

            // Store the logging id
            int id = entryIDs.get(field.getKey());

            // Send updates to NetworkTables and DataLog
            switch (field.getValue().type) {
                case Raw:
                    log.appendRaw(id, field.getValue().getRaw(), logTable.timestamp());
                    break;
                case Boolean:
                    log.appendBoolean(id, field.getValue().getBoolean(), logTable.timestamp());
                    break;
                case BooleanArray:
                    log.appendBooleanArray(id, field.getValue().getBooleanArray(), logTable.timestamp());
                    break;
                case Integer:
                    log.appendInteger(id, field.getValue().getInteger(), logTable.timestamp());
                    break;
                case IntegerArray:
                    log.appendIntegerArray(id, field.getValue().getIntegerArray(), logTable.timestamp());
                    break;
                case Float:
                    log.appendFloat(id, field.getValue().getFloat(), logTable.timestamp());
                    break;
                case FloatArray:
                    log.appendFloatArray(id, field.getValue().getFloatArray(), logTable.timestamp());
                    break;
                case Double:
                    log.appendDouble(id, field.getValue().getDouble(), logTable.timestamp());
                    break;
                case DoubleArray:
                    log.appendDoubleArray(id, field.getValue().getDoubleArray(), logTable.timestamp());
                    break;
                case String:
                    log.appendString(id, field.getValue().getString(), logTable.timestamp());
                    break;
                case StringArray:
                    log.appendStringArray(id, field.getValue().getStringArray(), logTable.timestamp());
                    break;
            }
        }
    }
}
