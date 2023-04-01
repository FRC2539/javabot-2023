package frc.lib.logging;

import frc.lib.logging.Logger.LogTable;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class LoggingThread extends Thread {
    private final BlockingQueue<LogTable> queue;

    private final List<Writer> writers;

    LoggingThread(BlockingQueue<LogTable> queue, List<Writer> writers) {
        super("Logging Thread");
        this.setDaemon(true);
        this.queue = queue;
        this.writers = writers;
    }

    public void run() {
        try {
            while (true) {
                // Await the next update
                LogTable updateTable = queue.take();

                // Send the update table to each writer
                writers.forEach((Writer writer) -> writer.sendUpdates(updateTable));
            }
        } catch (InterruptedException exception) {
            // Error printed in the main thread
        }
    }

    public interface Writer {
        public void sendUpdates(LogTable logTable);
    }
}
