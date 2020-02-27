/*
 *
 * reset.c
 *
 * Classical PI control servoing to zero the actuator positions.
 * Also, we can adapt this to reproduce the old gaits with the new
 * actuators if for some reason that becomes desirable. :) 
 *
 */
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

#include "libneurobot.h"

/* Default control constants k_p and k_i. */
#define DEFAULT_KI 6.f
#define DEFAULT_KP 3.f


int main(int argc, char**argv) 
{
    /*
     * Allow user specification of the control constants.
     */
    float k_p=DEFAULT_KP, k_i=DEFAULT_KI;

    int opt;
    char *endptr;
    while ((opt = getopt(argc, argv, "p:k:i:")) != -1) {
        if (opt == 'p') {
            set_pwm_max(strtod(optarg, &endptr));
            if (endptr && *endptr != '\0')
                die("Invalid PWM maximum", optarg);
        } else if (opt == 'k') {
            k_p = strtod(optarg, &endptr);
            if (endptr && *endptr != '\0') 
                die("Invalid feedback constant", optarg);
        } else if (opt == 'i') {
            k_i = strtod(optarg, &endptr);
            if (endptr && *endptr != '\0')
                die("Invalid feedback constant", optarg);
        } else die("Unrecognized argument", NULL);
    }

    /* 
     * If there's one argument left and it's a filename, write to it. 
     */
    if (optind+1 < argc) 
        die("Too many arguments!", NULL);
    if (optind+1 == argc) 
        open_logfile(argv[optind]);

    setup();
    float actuator_position[4];
    float interr[4] = {0, 0, 0, 0};

    datalogf("t,A0,A1,A2,A3,C0,C1,C2,C3\n");
    while (!g_please_die_kthxbai) {
        datalogf("%f", get_current_time());

        for (int i = 0; i < 4; ++i) {
            actuator_position[i] = read_adc(i);
            datalogf(", %f", actuator_position[i]);
        }

        for (int i = 0; i < 4; ++i) {
            /* Pretty conservative linear PI control. */
            float err = actuator_position[i] - 0.5;
            float control = -k_p*err - k_i*interr[i];
            interr[i] = 0.999*interr[i] + 0.001*err;
            datalogf(", %f", control);

            apply_actuator(i, control);
        }

        datalogf("\n");
        synchronize_loop();
    }

    print_final_time();
    cleanup();
}

