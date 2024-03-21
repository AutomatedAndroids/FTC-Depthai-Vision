package com.example.ftc_depth_ai.utils;

import com.qualcomm.robotcore.util.RobotLog;

public class AndroidLog {
    public static void androidLog(int prio, String tag, String fmt) {
        RobotLog.internalLog(prio,tag,fmt);
    }
}
