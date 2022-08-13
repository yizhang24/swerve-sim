package com.team1678.frc2022.logger;

public class LogEntry {

    private int target;
    private String[] values;

    public LogEntry(int target, String[] values){
        this.target = target;
        this.values = values;
    }

    public int getTarget() {
        return target;
    }

    public String[] getValues() {
        return values;
    }
}
