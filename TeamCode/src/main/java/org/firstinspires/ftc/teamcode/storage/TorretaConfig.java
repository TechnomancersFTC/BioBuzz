package org.firstinspires.ftc.teamcode.storage;

/**
 * Todas as constantes da torreta num lugar só.
 * Convenção de ângulo da torreta: 0° = frente do robô, positivo = anti-horário (visto de cima),
 * igual ao heading do Pedro Pathing.
 */
public final class TorretaConfig {
    private TorretaConfig() {}

    // ---------- Hardware (nomes no Robot Configuration) ----------
    public static final String SERVO_NAME   = "storreta2";   // Taura BL35 (modo rotação contínua / controle por potência)
    public static final String ENCODER_NAME = "encoder"; // porta de ENCODER (porta de motor) onde o Through Bore V2 está ligado

    // Se a torreta andar para o lado errado, troque o sinal aqui (engrenagens externas invertem o sentido!)
    public static final double SERVO_SIGN   = -1.0;
    public static final double ENCODER_SIGN = +1.0;

    // ---------- Relação de engrenagens ----------
    public static final double TURRET_TEETH  = 171;
    public static final double SERVO_TEETH   = 64;  // só afeta velocidade/torque, não a leitura de ângulo
    public static final double ENCODER_TEETH = 60;

    public static final double ENCODER_CPR = 8192;  // Through Bore V2 em quadratura (ticks por volta do eixo do encoder)

    // 1 volta da torreta = 171/60 voltas do encoder = 23347.2 ticks
    public static final double TICKS_PER_TURRET_REV = ENCODER_CPR * TURRET_TEETH / ENCODER_TEETH;
    public static final double DEG_PER_TICK = 360.0 / TICKS_PER_TURRET_REV; // ≈ 0.01542°/tick

    // ---------- Posição da torreta no robô (polegadas, a partir do centro de odometria do Pedro) ----------
    public static final double TURRET_OFFSET_FWD  = 0.0; // + = para frente
    public static final double TURRET_OFFSET_LEFT = 0.0; // + = para esquerda

    // ---------- Limites mecânicos (graus) — proteja os cabos! ----------
    public static final double MIN_ANGLE_DEG = -180;
    public static final double MAX_ANGLE_DEG = +180;

    // ---------- Alvos: centro de cada Hive no sistema de coordenadas do Pedro (0–144 pol) ----------
    public static final double HIVE_RED_X  = 57.38, HIVE_RED_Y  = 60.57;
    public static final double HIVE_BLUE_X = 84.5, HIVE_BLUE_Y = 80.22;

    // ---------- Controle (graus -> potência). Ajuste na ordem: kS, kP, kD, kV ----------
    public static final double kP = 0.015;  // potência por grau de erro
    public static final double kD = 0.0008; // amortecimento (usa velocidade medida)
    public static final double kS = 0.05;   // potência mínima para vencer o atrito
    public static final double kV = 0.0015; // feedforward: potência por (grau/s) de movimento do alvo
    public static final double MAX_POWER = 1.0;
    public static final double DEADBAND_DEG = 0.4;
}