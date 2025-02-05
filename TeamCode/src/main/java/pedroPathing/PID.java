package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

import java.util.List;
@Config
@TeleOp
public class PID extends OpMode {

    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0;
    public  static  double f = 0.2;

    public static int target = 0 ;

    private  final double ticks_in_degree = 2.77;

    private DcMotorEx  motor_stanga ; //the arm motor


    private PIDController hang1pid;
    public static double h1p = 0, h1i = 0, h1d = 0;
    public  static  double h1f = 0;

    public static int h1target = 0 ;

    private  final double h1ticks_in_degree = 3.434;

    private PIDController hang2pid;
    public static double h2p = 0, h2i = 0, h2d = 0;
    public  static  double h2f = 0;

    public static int h2target = 0 ;

    private  final double h2ticks_in_degree = 3.434;

    private PIDController lcontroller;
    public static double lp = 0.08, li = 0, ld = 0.00055;
    public  static  double lf = 0.15;

    public static int ltarget = 0 ;

    private  final double ticks_in_mm = 3.20;

    private DcMotorEx  motor_glisiere ; //the arm motor

    private DcMotorEx fata_stanga;
    private DcMotorEx fata_dreapta;
    private DcMotorEx spate_dreapta;
    private DcMotorEx spate_stanga;


    private DcMotorEx hang1 = null;
    private DcMotorEx hang2 = null;


    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    @Override
    public void init() {

        controller = new PIDController(p, i, d);
        hang1pid = new PIDController(h1p, h1i, h1d);
        hang2pid = new PIDController(h2p, h2i, h2d);
        lcontroller = new PIDController(lp, li, ld);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor_stanga = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");
        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");

        fata_dreapta =hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");

        motor_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);

        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fata_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        spate_stanga.setDirection(DcMotorSimple.Direction.REVERSE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        hang1pid.setPID(h1p, h1i, h1d);
        hang2pid.setPID(h2p, h2i, h2d);
        lcontroller.setPID(lp, li, ld);
        int armPos = motor_stanga.getCurrentPosition();
        int liftPos = motor_glisiere.getCurrentPosition();
        int hang1Pos = hang1.getCurrentPosition();
        int hang2Pos = hang2.getCurrentPosition();

        double pid = controller.calculate(armPos, target);
        double h1pid = hang1pid.calculate(hang1Pos, h1target);
        double h2pid = hang2pid.calculate(hang2Pos, h2target);
        double lpid = lcontroller.calculate(liftPos, ltarget);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double h1ff = Math.cos(Math.toRadians(h1target / h1ticks_in_degree)) * h1f;
        double h2ff = Math.cos(Math.toRadians(h2target / h2ticks_in_degree)) * h2f;
        double lff = Math.cos(Math.toRadians(ltarget / ticks_in_mm)) * lf;

        double power = pid + ff;
        double h1power = h1pid + h1ff;
        double h2power = h2pid + h2ff;
        double lpower = lpid + lff;

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fata_stanga.setPower(frontLeftPower);
        spate_stanga.setPower(backLeftPower);
        fata_dreapta.setPower(frontRightPower);
        spate_dreapta.setPower(backRightPower);



        motor_stanga.setPower(power);
        hang1.setPower(h1power);
        hang2.setPower(h2power);
        motor_glisiere.setPower(lpower);


        looptime = getRuntime();
        cycletime = looptime-oldtime;
        oldtime = looptime;

        telemetry.addData("pos arm", armPos);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("h1 pos", hang1Pos);
        telemetry.addData("h2 pos", hang2Pos);
        telemetry.update();
    }
}
