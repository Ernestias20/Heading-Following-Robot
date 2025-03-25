// DIRECTIVES DU PRE-PROCESSEUR
#include <18F45K22.h>
#use delay(crystal=8MHz)
#use i2c(Master, sda=PIN_C4, scl=PIN_C3) // pour l'afficheur et le module capteur Bussole
#use rs232(stream = capteur1, baud = 9600, PARITY=N, BITS=8, STOP=1, rcv=PIN_B0) // pour le capteur Ultrason
#use rs232(stream = capteur2, baud = 9600, PARITY=N, BITS=8, STOP=1, rcv=PIN_B5) // pour le capteur Ultrason
#include <lcd_I2C_1.c>

#define ADDR 0x4E // Adresse I2C du module LCD (peut varier)
#define LED PIN_A5
#define START_BUTTON PIN_A2
#define dir1_Motor1 PIN_D1 // sens horaire du moteur1
#define dir2_Motor1 PIN_D2 // sens anti-horaire moteur1
#define dir1_Motor2 PIN_D5 // sens anti-horaire moteur2
#define dir2_Motor2 PIN_D0 // sens horaire du moteur2

// S'assurer de l'addresse de l'afficheur sur la tramme du BUS I2C
#if ADDR != 0x4E
    #error LCD address must be 0x4E
#endif

// VARIABLES GLOBALES
unsigned int8 DATA[4];
unsigned int8 DAT[4];
unsigned int8 DATA1, i, Data2;
unsigned int8 angle, angle2, angle3;
unsigned int16 distCent, angleF, distCent2, cap;
unsigned int8 cent, diz, unit, cent2, diz2, unit2;
unsigned int8 control, control1, control2;
unsigned int16 speed_Motor1;
unsigned int16 speed_Motor2;
unsigned int16 gyroAngle;
unsigned int16 difference;

// FONCTIONS
void startMotor() {
    output_high(dir1_Motor1);
    output_high(dir2_Motor2);
    speed_Motor1 = 500;
    speed_Motor2 = 500;
    set_pwm1_duty(speed_Motor1);
    set_pwm2_duty(speed_Motor2);
}

void demiTourDroite() {
    output_high(dir1_Motor1);
    output_high(dir2_Motor2);
    speed_Motor1 = 500;
    speed_Motor2 = 0;
    set_pwm1_duty(speed_Motor1);
    set_pwm2_duty(speed_Motor2);
}

void demiTourGauche() {
    output_high(dir1_Motor1);
    output_high(dir2_Motor2);
    speed_Motor1 = 0;
    speed_Motor2 = 500;
    set_pwm1_duty(speed_Motor1);
    set_pwm2_duty(speed_Motor2);
}

void tournerDroite() {
    output_high(dir1_Motor1);
    output_high(dir2_Motor2);
    speed_Motor1 = 500;
    speed_Motor2 = 200;
    set_pwm1_duty(speed_Motor1);
    set_pwm2_duty(speed_Motor2);
}

void tournerGauche() {
    output_high(dir1_Motor1);
    output_high(dir2_Motor2);
    speed_Motor1 = 200;
    speed_Motor2 = 500;
    set_pwm1_duty(speed_Motor1);
    set_pwm2_duty(speed_Motor2);
}

void arreter() {
    output_low(dir1_Motor1);
    output_low(dir2_Motor2);
    speed_Motor1 = 0;
    speed_Motor2 = 0;
    set_pwm1_duty(speed_Motor1);
    set_pwm2_duty(speed_Motor2);
}

void suivreCap() {
    if (gyroAngle >= cap) {
        difference = gyroAngle - cap;
    } else {
        difference = cap - gyroAngle;
    }
    if (difference > 180) {
        difference = 360 - difference;
    }
    if (difference <= 5) {
        startMotor();
    } else {
        if ((cap > gyroAngle && cap - gyroAngle <= 180) || (cap < gyroAngle && gyroAngle - cap > 180)) {
            if (difference > 10) {
                demiTourDroite();
            } else {
                tournerDroite();
            }
        } else {
            if (difference > 10) {
                demiTourGauche();
            } else {
                tournerGauche();
            }
        }
    }
}

void startRobot() {
    if (input(START_BUTTON) && control == 1) {
        while (input(START_BUTTON)) {
            delay_ms(10);
        }
        control = 0;
    }
    if (control == 0 && control1 == 1) {
        output_high(LED);
        if (distCent <= 20 || distCent2 <= 20) {
            delay_ms(500);
            if (distCent < distCent2) {
                demiTourGauche();
            } else {
                demiTourDroite();
            }
        } else if (distCent <= 30 || distCent2 <= 30) {
            if (distCent < distCent2) {
                tournerGauche();
            } else {
                tournerDroite();
            }
        } else {
            suivreCap();
        }
    }
    if (control1 == 0) {
        cap = gyroAngle;
        control1 = 1;
        control = 1;
    }
}

#INT_TIMER0
void grabDistance_RB0() {
    // Lecture de la boussole et des capteurs
    i2c_start();
    i2c_write(0xc0);
    i2c_write(0x01);
    i2c_start();
    i2c_write(0xc1);
    angle = i2c_read(0);
    i2c_stop();
    delay_ms(1);

    LCD_goto(1, 2);
    printf(LCD_out, "A:%3lu°", gyroAngle);
}

void main() {
    set_pwm1_duty(0);
    set_pwm2_duty(0);
    setup_ccp1(CCP_PWM);
    setup_ccp2(CCP_PWM);
    setup_timer_2(T2_DIV_BY_16, 124, 1);
    setup_timer_0(T0_INTERNAL | T0_DIV_16);
    enable_interrupts(global);
    enable_interrupts(INT_TIMER0);
    set_timer0(60000);
    control = 1;
    control1 = 0;
    control2 = 0;
    LCD_begin(ADDR);
    cap = 0;

    while (TRUE) {
        startRobot();
    }
}
