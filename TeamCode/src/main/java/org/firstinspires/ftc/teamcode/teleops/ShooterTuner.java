//package org.firstinspires.ftc.teamcode.teleops;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//
//import dev.nextftc.control.feedback.PIDController;
//import dev.nextftc.control.feedback.PIDCoefficients;
//
//@TeleOp(name = "Shooter Tuner", group = "Teste")
//public class ShooterTuner extends LinearOpMode {
//
//    // ===== Hardware =====
//    public static String MOTOR_1 = "shooter";
//    public static String MOTOR_2 = "";          // deixe vazio se só tem 1 motor
//    public static boolean REVERSE_1 = false;
//    public static boolean REVERSE_2 = false;
//
//    // ===== PID + Feedforward (NextFTC) =====
//    public static double kP = 0.0;
//    public static double kI = 0.0;
//    public static double kD = 0.0;
//    public static double kV = 0.0;           // feedforward: potência por (tick/s)
//    public static double kS = 0.0;              // feedforward: potência para vencer atrito
//
//    // ===== Alvo =====
//    public static boolean MODO_TABELA = false;
//    public static double ALVO_TICKS = 1500;     // usado quando MODO_TABELA = false
//    public static double DISTANCIA = 60;        // usado quando MODO_TABELA = true
//    public static double PASSO_DISTANCIA = 2;
//    public static double TOLERANCIA_TICKS = 40; // para "pronto p/ atirar"
//
//    // ===== Tabela: distância (pol) x ticks/s  (distâncias em ordem crescente) =====
//    // Ponto com distância e ticks = 0 é ignorado (use só quantos precisar).
//    public static double D1 = 30,  T1 = 1200;
//    public static double D2 = 50,  T2 = 1400;
//    public static double D3 = 70,  T3 = 1600;
//    public static double D4 = 90,  T4 = 1800;
//    public static double D5 = 0,   T5 = 0;
//    public static double D6 = 0,   T6 = 0;
//
//    @Override
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, MOTOR_1);
//        DcMotorEx m2 = MOTOR_2.isEmpty() ? null : hardwareMap.get(DcMotorEx.class, MOTOR_2);
//
//        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        if (m2 != null) {
//            m2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        }
//
//        // O ControlSystem é recriado só quando você muda algum ganho no Dashboard
//        double[] ganhos = {kP, kI, kD};
//        PIDController controle = criarControle();
//
//        boolean ligado = false, aAnt = false, upAnt = false, downAnt = false;
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // --- gamepad ---
//            boolean a = gamepad1.a, up = gamepad1.dpad_up, down = gamepad1.dpad_down;
//            if (a && !aAnt) ligado = !ligado;
//            if (up && !upAnt) DISTANCIA += PASSO_DISTANCIA;
//            if (down && !downAnt) DISTANCIA -= PASSO_DISTANCIA;
//            aAnt = a; upAnt = up; downAnt = down;
//
//            // --- recria o controle se algum ganho mudou ---
//            double[] atuais = {kP, kI, kD};
//            if (!java.util.Arrays.equals(ganhos, atuais)) {
//                ganhos = atuais;
//                controle = criarControle();
//            }
//
//            // --- alvo ---
//            double alvo = MODO_TABELA ? ticksParaDistancia(DISTANCIA) : ALVO_TICKS;
//            double velocidade = Math.abs(m1.getVelocity());   // ticks/s
//
//            double potencia = 0;
//            if (ligado) {
//                double feedforward = kV * alvo + kS;
//                double feedback = controle.calculate(alvo - velocidade);   // erro = alvo - atual
//                potencia = Math.max(0, Math.min(1, feedforward + feedback));
//            } else {
//                controle = criarControle();   // zera a integral enquanto desligado
//            }
//
//            m1.setPower(REVERSE_1 ? -potencia : potencia);
//            if (m2 != null) m2.setPower(REVERSE_2 ? -potencia : potencia);
//
//            // --- telemetria (gráfico: alvo x atual) ---
//            telemetry.addData("Ligado", ligado);
//            telemetry.addData("Modo", MODO_TABELA ? "TABELA" : "MANUAL");
//            telemetry.addData("Distancia", DISTANCIA);
//            telemetry.addData("alvo", ligado ? alvo : 0);
//            telemetry.addData("atual", velocidade);
//            telemetry.addData("erro", ligado ? alvo - velocidade : 0);
//            telemetry.addData("potencia", potencia);
//            telemetry.addData("Pronto p/ atirar", ligado && Math.abs(alvo - velocidade) <= TOLERANCIA_TICKS);
//            telemetry.update();
//        }
//
//        m1.setPower(0);
//        if (m2 != null) m2.setPower(0);
//    }
//
//    private PIDController criarControle() {
//        return new PIDController(new PIDCoefficients(kP, kI, kD));
//    }
//
//    /** Interpolação linear distância -> ticks/s. Fora da faixa, trava no primeiro/último ponto. */
//    private double ticksParaDistancia(double dist) {
//        double[] d = {D1, D2, D3, D4, D5, D6};
//        double[] t = {T1, T2, T3, T4, T5, T6};
//
//        // usa só os pontos preenchidos
//        int n = 0;
//        double[] dd = new double[6], tt = new double[6];
//        for (int i = 0; i < 6; i++) {
//            if (d[i] != 0 || t[i] != 0) { dd[n] = d[i]; tt[n] = t[i]; n++; }
//        }
//        if (n == 0) return 0;
//        if (dist <= dd[0]) return tt[0];
//        if (dist >= dd[n - 1]) return tt[n - 1];
//
//        for (int i = 0; i < n - 1; i++) {
//            if (dist >= dd[i] && dist <= dd[i + 1]) {
//                double f = (dist - dd[i]) / (dd[i + 1] - dd[i]);
//                return tt[i] + f * (tt[i + 1] - tt[i]);
//            }
//        }
//        return tt[n - 1];
//    }
//}