#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <stdarg.h>

#include <libpruio/pruio.h>
#include <libpruio/pruio_pins.h>

/* Safety feature: max PWM duty cycle. */
#define PWM_MAX 0.2

/* Constant PWM frequency. */
#define PWM_FREQ_HZ 200.f

const int pwm_pins[4] = {
    P9_31, P9_29, P9_14, P9_16
};

const int gpio_pins[4] = {
    P8_07, P8_08, P8_10, P8_09
};

void die(const char *message, const char *error)
{
    if (error) fprintf(stderr, "%s: %s\n", message, error);
    else fprintf(stderr, "%s :(\n", message);
    exit(1);
}

pruIo *g_pru = NULL;
uint8_t g_pinmodes[4] = {};
void cleanup() 
{
    /* Zero all the PWMs first because if left nonzero, they will do
     * horrible things. */
    for (int i = 0; i < 4; i++) {
        if (pruio_pwm_setValue(g_pru, pwm_pins[i], -1, 0))
            die("Couldn't set PWM", g_pru->Errr);
    }

    /* Sleep for 100ms to leave some space to shut down. */
    usleep(100000);

    /* Reset the PRU state */
    pruio_destroy(g_pru);

    fprintf(stderr, "Cleaned up. :)\n");
}

bool g_please_die_kthxbai = false;
void die_gracefully(int signal)
{
    (void)signal;
    g_please_die_kthxbai = true;
    fprintf(stderr, "Caught signal, exiting.\n");
}

FILE *g_logprintfile = NULL;
int logprintf(const char *fmt, ...) 
{
    if (g_logprintfile == NULL) return 0;

    va_list args;
    va_start(args, fmt);
    int ret = vfprintf(g_logprintfile, fmt, args);
    va_end(args);
    return ret;
}

int main(int argc, char **argv) 
{
    /* 
     * If there's one argument and it's a filename, write to it. 
     */
    if (argc > 2) die("Too many arguments.", NULL);
    if (argc == 2) {
        if (!strncmp("-", argv[1], 2)) g_logprintfile = stdout;
        else g_logprintfile = fopen(argv[1], "w");

        if (g_logprintfile == NULL) {
            perror("Couldn't open logprintfile");
            exit(1);
        }
    }


    /*
     * Create the device driver object. 
     * The first parameter is a 16-bit mask specifying which 
     * subsystems to activate; here, we turn them all on.
     * Next is an exponential moving-average filter time 
     * constant, in sample numbers, followed by the delay
     * in cycles between configuration and the start of
     * ADC readings; finally, a sample delay describing
     * how long to wait between reading the ADC. It's probably
     * wasteful to leave this at 0 considering how slowly we
     * sample, but for now it'll do...
     */
    g_pru = pruio_new(PRUIO_DEF_ACTIVE, 4, 0x98, 0);
    if (!g_pru) {
        perror(NULL);
        exit(1);
    }

    signal(SIGTERM, die_gracefully);
    signal(SIGINT, die_gracefully);
    if (g_pru->Errr) 
        die("PruIO initialization failed", g_pru->Errr);

    if (g_pru->Errr) 
        die("PruIO initialization failed", g_pru->Errr);

    /*
     * Save a constant necessary for correctly setting GPIOs.
     * I still don't know why it works this way, but for 
     * some reason, you have to set the pin to pinmode|128 
     * to turn it on, or just pinmode to turn it off.
     */
    for (int i = 0; i < 4; i++) {
        g_pinmodes[i] = g_pru->BallConf[gpio_pins[i]];
    }

    /*
     * Initialize the four PWM pins corresponding to the motor enable
     * lines. These run at 200Hz because there's too much parasitic
     * inductance in those magnet wires, and begin at 0% duty cycle.
     */
    for (int i = 0; i < 4; i++) { if (pruio_pwm_setValue(g_pru,
                pwm_pins[i], PWM_FREQ_HZ, 0)) die("Couldn't set PWM",
                g_pru->Errr); }

    /*
     * Send the config to the PRU. The parameters set the
     * driver to IO mode (i.e. sampling on demand), activate
     * the four ADC channels we're actually using, give zero
     * sampling frequency because that's not used in IO mode,
     * and say to return raw 12-bit values.
     */
    if (pruio_config(g_pru, 1, 0xF<<1, 0, 0)) 
        die("Config failed", g_pru->Errr);
    
    /* Actuator position to control to 0.5. */
    logprintf("t,A0,A1,A2,A3,C0,C1,C2,C3\n");
    long timesteps = 0;
    float actuator_position[4];
    float interr[4] = {0, 0, 0, 0};
    while (!g_please_die_kthxbai) {
        logprintf("%d", timesteps++);

        /* 
         * Read all the actuator positions, taking a 12-bit ADC
         * value and converting it to a floating-point number
         * in the interval [0,1]. 
         */
        for (int i = 0; i < 4; ++i) {
            uint16_t raw = g_pru->Adc->Value[i+1];
            actuator_position[i] = 1.f*raw / (1<<12);
            logprintf(", %f", actuator_position[i]);
        }

        /*
         * Pretty conservative linear feedback control.
         */
        for (int i = 0; i < 4; i++) {

            /* 
             * Desired actuator position: 0.5.
             */
            float err = actuator_position[i] - 0.5;
            float control = -3*err - 6*interr[i];
            if (control > PWM_MAX) control = PWM_MAX;
            if (control < -PWM_MAX) control = -PWM_MAX;
            interr[i] = 0.999*interr[i] + 0.001*err;

            logprintf(", %f", control);

            /* 
             * Set the PWM duty cycle. The argument of -1 says to keep the
             * frequency the same.
             */
            if (pruio_pwm_setValue(g_pru, pwm_pins[i], -1, 
                        fabs(control)))
                die("Couldn't set PWM", g_pru->Errr);

            /* 
             * Set the direction with a sign. 
             * I have no idea which way it should go...
             */
            bool is_negative = signbit(control);
            int mask = (is_negative?0:128) | g_pinmodes[i];
            if (pruio_gpio_setValue(g_pru, gpio_pins[i], mask))
                die("Couldn't do GPIO", g_pru->Errr);
        }
        logprintf("\n");

        usleep(1000);
    }

    cleanup();
}

