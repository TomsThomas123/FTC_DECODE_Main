package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo claw;

    public Wrist(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "wrist");
    }
    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(0);
            return false;
        }
    }
    public Action wristDown() {
        return new WristDown();
    }

}