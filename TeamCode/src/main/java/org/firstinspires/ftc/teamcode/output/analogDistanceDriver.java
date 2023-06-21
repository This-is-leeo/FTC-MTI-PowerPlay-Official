package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class analogDistanceDriver {

    private AnalogInput analog;
    private int maxR = 520;

    public analogDistanceDriver(AnalogInput analog) {
        this.analog = analog;
    }

    public double getDistance() {
        double distance = (this.analog.getVoltage()*maxR)/3.3;
        return distance;
    }


}