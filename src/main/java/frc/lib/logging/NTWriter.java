package frc.lib.logging;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.lib.logging.Logger.LogTable;
import frc.lib.logging.LoggingThread.Writer;
import java.util.HashMap;
import java.util.Map;

public class NTWriter implements Writer {
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private final Map<String, GenericPublisher> publishers = new HashMap<>();

    public void sendUpdates(LogTable logTable) {
        for (Map.Entry<String, LogValue> field : logTable.updatesMap().entrySet()) {
            // Skip any values with NT logging disabled
            if (field.getValue().sendToNT) {

                // Create publisher if necessary
                String key = field.getKey();
                GenericPublisher publisher = publishers.get(key);
                if (publisher == null) {
                    publisher = nt.getTopic(key)
                            .genericPublish(field.getValue().type.getNT4Type(), PubSubOption.sendAll(true));
                    publishers.put(key, publisher);
                }

                // Send updates to NetworkTables and DataLog
                switch (field.getValue().type) {
                    case Raw:
                        publisher.setRaw(field.getValue().getRaw(), logTable.timestamp());
                        break;
                    case Boolean:
                        publisher.setBoolean(field.getValue().getBoolean(), logTable.timestamp());
                        break;
                    case BooleanArray:
                        publisher.setBooleanArray(field.getValue().getBooleanArray(), logTable.timestamp());
                        break;
                    case Integer:
                        publisher.setInteger(field.getValue().getInteger(), logTable.timestamp());
                        break;
                    case IntegerArray:
                        publisher.setIntegerArray(field.getValue().getIntegerArray(), logTable.timestamp());
                        break;
                    case Float:
                        publisher.setFloat(field.getValue().getFloat(), logTable.timestamp());
                        break;
                    case FloatArray:
                        publisher.setFloatArray(field.getValue().getFloatArray(), logTable.timestamp());
                        break;
                    case Double:
                        publisher.setDouble(field.getValue().getDouble(), logTable.timestamp());
                        break;
                    case DoubleArray:
                        publisher.setDoubleArray(field.getValue().getDoubleArray(), logTable.timestamp());
                        break;
                    case String:
                        publisher.setString(field.getValue().getString(), logTable.timestamp());
                        break;
                    case StringArray:
                        publisher.setStringArray(field.getValue().getStringArray(), logTable.timestamp());
                        break;
                }
            }
        }
    }
}
