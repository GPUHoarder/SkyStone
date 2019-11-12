package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Configure {
    private Trobot trobot = new Trobot();
    private HardwareMap hardwareMap = null;

    Configure(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Trobot getTrobot() {
        return trobot;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void Initialize() {
        InitializeDrive();
        InitializeServo();
        InitializeCollector();
    }

    public void InitializeDrive() {
        trobot.setLeftDriveFront(hardwareMap.get(DcMotor.class, "front left"));
        trobot.setRightDriveFront(hardwareMap.get(DcMotor.class, "front right"));
        trobot.setLeftDriveBack(hardwareMap.get(DcMotor.class, "rear left"));
        trobot.setRightDriveBack(hardwareMap.get(DcMotor.class, "rear right"));

        trobot.getLeftDriveFront().setDirection(DcMotor.Direction.REVERSE);
        trobot.getRightDriveFront().setDirection(DcMotor.Direction.FORWARD);
        trobot.getLeftDriveBack().setDirection(DcMotor.Direction.REVERSE);
        trobot.getRightDriveBack().setDirection(DcMotor.Direction.FORWARD);
    }

    public void InitializeServo() {
        trobot.setLeftServo(hardwareMap.get(Servo.class, "left servo"));
        trobot.setRightServo(hardwareMap.get(Servo.class, "right servo"));
    }

    public void InitializeCollector() {
        trobot.setIntakeLeft(hardwareMap.get(DcMotor.class, "left intake"));
        trobot.setIntakeRight(hardwareMap.get(DcMotor.class, "right intake"));

        trobot.getIntakeLeft().setDirection(DcMotor.Direction.FORWARD);
        trobot.getIntakeRight().setDirection(DcMotor.Direction.REVERSE);
    }

}