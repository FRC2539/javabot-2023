package frc.lib.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.logging.LogValue.LoggableType;
import frc.lib.logging.Logger.LogTable;

public class LoggingThread extends Thread {
    private final BlockingQueue<LogTable> queue;

    private final Map<String, GenericPublisher> publishers = new HashMap<>();
    private final Map<String, Integer> entryIDs = new HashMap<>();

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final DataLog log = DataLogManager.getLog();

    LoggingThread(BlockingQueue<LogTable> queue) {
        super("Logging Thread");
        this.setDaemon(true);
        this.queue = queue;
    }

    public void run() {
        try {
            while (true) {
                // Await the next update
                LogTable updateTable = queue.take();

                sendUpdates(updateTable);
            }
        } catch (InterruptedException exception) {
            // Error printed in the main thread
        }
    }

    private void sendUpdates(LogTable logTable) {
        for (Map.Entry<String, LogValue> field : logTable.updatesMap().entrySet()) {
            // Create publisher if necessary
            String key = field.getKey();
            GenericPublisher publisher = publishers.get(key);
            if (publisher == null) {
              publisher = nt.getTopic(key).genericPublish(field.getValue().type.getNT4Type(),
                  PubSubOption.sendAll(true));
              publishers.put(key, publisher);
            }

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
                publisher.setRaw(field.getValue().getRaw(), logTable.timestamp());
                log.appendRaw(id, field.getValue().getRaw(), logTable.timestamp());
                break;
              case Boolean:
                publisher.setBoolean(field.getValue().getBoolean(), logTable.timestamp());
                log.appendBoolean(id, field.getValue().getBoolean(), logTable.timestamp());
                break;
              case BooleanArray:
                publisher.setBooleanArray(field.getValue().getBooleanArray(), logTable.timestamp());
                log.appendBooleanArray(id, field.getValue().getBooleanArray(), logTable.timestamp());
                break;
              case Integer:
                publisher.setInteger(field.getValue().getInteger(), logTable.timestamp());
                log.appendInteger(id, field.getValue().getInteger(), logTable.timestamp());
                break;
              case IntegerArray:
                publisher.setIntegerArray(field.getValue().getIntegerArray(), logTable.timestamp());
                log.appendIntegerArray(id, field.getValue().getIntegerArray(), logTable.timestamp());
                break;
              case Float:
                publisher.setFloat(field.getValue().getFloat(), logTable.timestamp());
                log.appendFloat(id, field.getValue().getFloat(), logTable.timestamp());
                break;
              case FloatArray:
                publisher.setFloatArray(field.getValue().getFloatArray(), logTable.timestamp());
                log.appendFloatArray(id, field.getValue().getFloatArray(), logTable.timestamp());
                break;
              case Double:
                publisher.setDouble(field.getValue().getDouble(), logTable.timestamp());
                log.appendDouble(id, field.getValue().getDouble(), logTable.timestamp());
                break;
              case DoubleArray:
                publisher.setDoubleArray(field.getValue().getDoubleArray(), logTable.timestamp());
                log.appendDoubleArray(id, field.getValue().getDoubleArray(), logTable.timestamp());
                break;
              case String:
                publisher.setString(field.getValue().getString(), logTable.timestamp());
                log.appendString(id, field.getValue().getString(), logTable.timestamp());
                break;
              case StringArray:
                publisher.setStringArray(field.getValue().getStringArray(), logTable.timestamp());
                log.appendStringArray(id, field.getValue().getStringArray(), logTable.timestamp());
                break;
            }
          }
    }
}
