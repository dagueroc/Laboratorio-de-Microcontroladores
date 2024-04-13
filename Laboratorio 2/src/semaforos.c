/*
Universidad de Costa Rica 
Laboratorio de Microcontroladores

Autores:
Noel Blandon S.
Dylan Aguero C.

Fecha:12/4/2024
*/

#include <avr/io.h>
#include <avr/interrupt.h>

/* MACROS */
#define RESET_CICLOS_TEMPORIZADOR 0
#define UN_SEGUNDO 1
#define TRES_SEGUNDOS 3
#define DIEZ_SEGUNDOS 10
#define CICLO_MEDIO_SEGUNDO 30
#define CICLO_SEGUNDO_COMPLETO 60
#define RESET_CICLO_SEGUNDO_COMPLETO 63

/* ESTADOS */
typedef enum {
    PASO_VEHICULO,
    PARPADEO_VEHICULO,
    DETENER_VEHICULO,
    PASO_PEATON,
    PARPADEO_PEATON,
    DETENER_PEATON
} Estado;

Estado estado_actual;

/* VARIABLES GLOBALES */
int pulsado = 0;
int ciclos_temporizador = 0;
int segundos = 0;

/* DECLARACIÓN DE FUNCIONES */
void inicializar();
void maquina_estados();
inline void rutina_interrupcion();
inline void rutina_parpadeo();
inline void reiniciar_tiempo_y_ciclos();

/* INTERRUPCIONES */
ISR(INT0_vect){
    pulsado = 1;
}

ISR(TIMER0_OVF_vect){
    rutina_interrupcion();
}

/* MAIN */
int main(void) {
    inicializar();
    sei();

    while (1) {
        maquina_estados();
    }
}

/* FUNCIONES */

void inicializar(){
    DDRB |= (1 << PB3)|(1 << PB2)|(1 << PB1)|(1 << PB0);
    PORTB = (0<<PB3)|(1<<PB2)|(0<<PB1)|(1<<PB0);
    estado_actual = PASO_VEHICULO;
    pulsado = 0;
    segundos = 0;
    GIMSK |= (1<<INT0);
    MCUCR |= (1 << ISC00) | (1 << ISC01);
    TCCR0A = 0x00;
    TCCR0B = (1 << CS00) | (1 << CS02);
    TCNT0 = 0;
    TIMSK |= (1 << TOIE0);
}

void maquina_estados(){
    switch (estado_actual){
        case (PASO_VEHICULO):
            PORTB = (0<<PB3)|(1<<PB2)|(0<<PB1)|(1<<PB0);
            if((pulsado == 1) && (segundos >= DIEZ_SEGUNDOS)){
                reiniciar_tiempo_y_ciclos();
                estado_actual = PARPADEO_VEHICULO;
            }
            break;

        case (PARPADEO_VEHICULO):
            if((segundos >= TRES_SEGUNDOS) && (ciclos_temporizador == CICLO_MEDIO_SEGUNDO || ciclos_temporizador == CICLO_SEGUNDO_COMPLETO)){
                reiniciar_tiempo_y_ciclos();
                estado_actual = DETENER_VEHICULO;
            }
            break;

        case (DETENER_VEHICULO):
            PORTB = (0<<PB3)|(1<<PB2)|(1<<PB1)|(0<<PB0);
            if (segundos >= UN_SEGUNDO){
                reiniciar_tiempo_y_ciclos();
                estado_actual = PASO_PEATON;
            }
            break;

        case (PASO_PEATON):
            PORTB = (1<<PB3)|(0<<PB2)|(1<<PB1)|(0<<PB0);
            if(segundos >= DIEZ_SEGUNDOS){
                reiniciar_tiempo_y_ciclos();
                estado_actual = PARPADEO_PEATON;
            }
            break;

        case (PARPADEO_PEATON):
            if((segundos >= TRES_SEGUNDOS) && (ciclos_temporizador == CICLO_MEDIO_SEGUNDO || ciclos_temporizador == CICLO_SEGUNDO_COMPLETO)){
                reiniciar_tiempo_y_ciclos();
                estado_actual = DETENER_PEATON;
            }
            break;

        case (DETENER_PEATON):
            PORTB = (0<<PB3)|(1<<PB2)|(1<<PB1)|(0<<PB0);
            if(segundos >= UN_SEGUNDO){
                reiniciar_tiempo_y_ciclos();
                pulsado = 0;
                estado_actual = PASO_VEHICULO;
            }
            break;

        default:
            estado_actual = PASO_VEHICULO;
            break;
    }
}

inline void rutina_parpadeo(){
    switch (estado_actual){
        case PARPADEO_VEHICULO:
            if(ciclos_temporizador == CICLO_MEDIO_SEGUNDO || ciclos_temporizador == CICLO_SEGUNDO_COMPLETO){
                PORTB ^= (1<<PB0);
            }
            break;

        case PARPADEO_PEATON:
            if(ciclos_temporizador == CICLO_MEDIO_SEGUNDO || ciclos_temporizador == CICLO_SEGUNDO_COMPLETO){
                PORTB ^= (1<<PB3);
            }
            break;

        default:
            break;
    }
}

inline void rutina_interrupcion(){
    rutina_parpadeo();
    if(ciclos_temporizador == RESET_CICLO_SEGUNDO_COMPLETO){
        segundos++;
        ciclos_temporizador = RESET_CICLOS_TEMPORIZADOR;
    } else {
        ciclos_temporizador++;
    }
}

inline void reiniciar_tiempo_y_ciclos() {
    ciclos_temporizador = RESET_CICLOS_TEMPORIZADOR;
    segundos = RESET_CICLOS_TEMPORIZADOR;
}
