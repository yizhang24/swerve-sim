package com.team1678.frc2022.logger;

import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.sql.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.TimeZone;
import java.util.function.Supplier;

import com.team1678.frc2022.Robot;

import java.lang.Object;

import static java.util.Map.entry;

public class LoggingSystem {

    // Constants
    private static final int kQueueCapacity = 1000001;
    public final String kRootDirectory;
    
    public final LogWriter mLogWriter;

    private boolean running = false;
    private static ArrayList<ArrayList<Supplier<String>>> mElements = new ArrayList<ArrayList<Supplier<String>>>();
    private static ArrayList<LogStorage> mStorage = new ArrayList<LogStorage>();
    private static ArrayDeque<LogEntry> mQueue = new ArrayDeque<LogEntry>(kQueueCapacity);
    private static File mLogDirectory = null;

    private static DateFormat dateFormat = new SimpleDateFormat(
            "dd_MMM_yy 'at' hh.mm.ss aa");


    private Date startTime;
    private static boolean inCompetition = false;
    private static String eventName;
    private static String matchType;
    private static Integer matchNumber;

    private final boolean isBenchmark;

    public LoggingSystem(boolean benchmarking) {
        mLogWriter = new LogWriter(mQueue);
        dateFormat.setTimeZone(TimeZone.getTimeZone("PST"));
        isBenchmark = benchmarking;
        if (isBenchmark || Robot.isSimulation()) {
            kRootDirectory = "./Output Logs";
        } else {
            kRootDirectory = "/home/lvuser/logs";
        }
    }

    public synchronized int registerObject(Class<?> loggedClass, Object loggedObject) {

        ArrayList<String> headers = new ArrayList<String>();

        int subsystemIndex = mElements.size();
        mElements.add(new ArrayList<Supplier<String>>());

        for (Method method : loggedClass.getDeclaredMethods()) {
            if (!method.isAnnotationPresent(Log.class) || !LoggalbeHandler.containsKey(method.getReturnType())
                    || method.getParameterCount() > 0) {
                continue;
            }
            method.setAccessible(true);
            LoggableProcessor processor = LoggalbeHandler.get(method.getReturnType());
            processor.getLoggalbe(() -> {
                try {
                    return method.invoke(loggedObject);
                } catch (IllegalAccessException | InvocationTargetException e) {
                    e.printStackTrace();
                    return null;
                }
            }, subsystemIndex);
            headers.add(method.getName());
        }

        for (Field field : loggedClass.getDeclaredFields()) {
            if (!field.isAnnotationPresent(Log.class) || !LoggalbeHandler.containsKey(field.getType())) {
                continue;
            }
            field.setAccessible(true);
            LoggableProcessor processor = LoggalbeHandler.get(field.getType());
            processor.getLoggalbe(() -> {
                try {
                    return field.get(loggedObject);
                } catch (IllegalAccessException e) {
                    return null;
                }
            }, subsystemIndex);
            headers.add(field.getName());
        }

        String name;
        if (isBenchmark) {
            name = "ANNOTATION_LOGGER";
        } else {
            name = loggedClass.getSimpleName();
        }
        LogStorage store = new LogStorage(name, headers);
        mStorage.add(store);
        return subsystemIndex;
    }


    public void setDirectory() {

        String path = kRootDirectory;

        if (!isBenchmark || Robot.isReal()) {
            // create logs folder if not present
            File rootDirectory = new File(kRootDirectory);
            if (!rootDirectory.isDirectory()) {
                rootDirectory.mkdir();
            }

            Integer maxNum = 0;
            for (final File entry : rootDirectory.listFiles()) {
                try {
                    if (!entry.isDirectory()) {
                        continue;
                    }
                    String directory_name = entry.getName();
                    int char_index = directory_name.indexOf(")");
                    int num = Integer.parseInt(directory_name.substring(1, char_index));
                    if (num > maxNum) {
                        maxNum = num;
                    }
                } catch (Exception e) {
                    // Files that are not numbers are expected and ignored
                }
            }
            maxNum++;

            // get system time in milliseconds and convert to datetime in PST time zone
            startTime = new Date(System.currentTimeMillis());

            // format time in datetime and add to file name
            path = kRootDirectory + "/(" + maxNum.toString() + ") " + dateFormat.format(startTime);

            // create new directory
            mLogDirectory = new File(path);
            mLogDirectory.mkdir();
        }

        // create new directory
        mLogDirectory = new File(path);
        mLogDirectory.mkdir();

        // update filewriters
        for (int i = 0; i < mStorage.size(); i++) {
            mStorage.get(i).setPath(path);
        }
        
    }

    public void queueLogs() {
        for (int i = 0; i < mElements.size(); i++) {
            ArrayList<Supplier<String>> values = mElements.get(i);
            String[] temp = new String[values.size()];
            for (int j = 0; j < values.size(); j++) {
                temp[j] = values.get(j).get();
            }
            mQueue.add(new LogEntry(i, temp));
        }
    }

    public int queueSize() {
        return mQueue.size();
    }

    public void start() {
        if (!running) {
            running = true;
            setDirectory();
            mLogWriter.updateStorage(mStorage);
            System.out.println("Starting Logger");
            if (mLogWriter.isAlive()) {
                mLogWriter.interrupt();
            }
            mLogWriter.start();
        }
    }

    public boolean isDone() {
        return mQueue.isEmpty();
    }

    public void end() {
        if (running) {
            running = false;

            mLogWriter.end();

            if (inCompetition) {
                try {
                    String path = kRootDirectory + "/" + eventName + matchType + " Match "
                            + matchNumber.toString() + "_" + dateFormat.format(startTime);
                    mLogDirectory.renameTo(new File(path));  
                } catch (NoSuchElementException e) {
                    // No-op
                }

            }

        }
    }

    public void reset() {
        mLogWriter.close();
        mQueue.clear();
        mStorage.clear();
        mElements.clear();
        mLogWriter.updateStorage(mStorage);
        mLogDirectory = null;
    }

    public void updateStorage() {
        mLogWriter.updateStorage(mStorage);
    }
    
    public void updateMatchInfo(String newEventName, String newMatchType, int newMatchNumber) {
        if (inCompetition) {
            return; // We already have match info
        }

        inCompetition = true;
        eventName = newEventName;
        matchType = newMatchType;
        matchNumber = newMatchNumber;
    }

    private static LoggingSystem mInstance;

    public static LoggingSystem getInstance() {
        if (mInstance == null) {
            mInstance = new LoggingSystem(false);
        }
        return mInstance;
    }

    @FunctionalInterface
    public interface LoggableProcessor {
        public void getLoggalbe(Supplier<Object> supplier, int subsystem);
    }

    public static Map<Class<?>, LoggableProcessor> LoggalbeHandler = Map.ofEntries(
            entry(int.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            return String.valueOf(supplier.get());
                        });
                    }),
            entry(boolean.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            return String.valueOf(supplier.get());
                        });
                    }),
            entry(String.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            return (String) supplier.get();
                        });
                    }),
            entry(double.class,
                    (supplier, subsystem) -> {
                        mElements.get(subsystem).add(() -> {
                            if (supplier.get() == null) {
                                return new String();
                            }
                            return String.valueOf(supplier.get());
                        });
                    }));

}
