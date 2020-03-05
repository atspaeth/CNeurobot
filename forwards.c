/*
 *
 * forwards.c
 *
 * The basic forward-only controller which produces the results reported
 * in the RoboSoft paper. 
 *
 */

#include "libneurobot.h"

#include "forwards.h"

/* The strength of position feedback in pA. */
#define DEFAULT_FEEDBACK 25

int main(int argc, char**argv) 
{

    /*
     * Parsing command-line options.
     */
    float feedback = DEFAULT_FEEDBACK;

    int opt;
    char *endptr;
    while ((opt = getopt(argc, argv, "p:k:")) != -1) {
        if (opt == 'p') {
            set_pwm_max(strtod(optarg, &endptr));
            if (endptr && *endptr != '\0')
                die("Invalid PWM maximum", optarg);
        } else if (opt == 'k') {
            feedback = strtod(optarg, &endptr);
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

    states[0].v = 0;
    datalogf("t,A0,A1,A2,A3");
    for (int i=0; i<N_CELLS; i++)
        datalogf(",V%d", i);
    datalogf("\n");
    while (!g_please_die_kthxbai) {
        datalogf("%f", get_current_time());

        for (int i = 0; i < 4; ++i) {
            actuator_position[i] = read_adc(i);
            datalogf(", %f", actuator_position[i]);
        }

        /* 
         * Need to check all spikes before doing any dynamics for
         * consistency with the Python version. 
         */
        for (int i = 0; i < N_CELLS; i++) {
            check_spike(&states[i], params[i]);
        }

        /* 
         * Now run the continuous dynamics, with input currents
         * calculated both for the synapses and for feedback.
         */
        for (int i = 0; i < N_CELLS; i++) {
            float i_in = 0;

            /* 
             * Compute synaptic current.
             */
            for (int j = 0; j < N_CELLS; j++) {
                float deltaV = params[j]->vn - states[i].v;
                i_in += G[i][j] * deltaV * states[j].i;
            }

            /* Compute feedback current. */
            if (i < N_CELLS-4 && (i%3)==0) {
                int prev = (i/3+3)%4;
                int next = (i/3+1)%4;
                
                float prev_err = fabs(1 - actuator_position[prev]);
                float next_err = fabs(0 - actuator_position[next]);
                i_in += -feedback*(prev_err + next_err);
            }

            resolve_dynamics(&states[i], params[i], i_in);

            datalogf(", %f", states[i].v);
        }

        /* This part actually communicates with the motor. */
        for (int i = 0; i < 4; i++) {
            int flexor = i + N_CELLS-4;
            int extensor = (i + 2)%4 + N_CELLS-4;
            float activation = states[flexor].v - states[extensor].v;

            apply_actuator(i, activation);
        }

        datalogf("\n");
        synchronize_loop();
    }

    print_final_time();
    cleanup();
}

