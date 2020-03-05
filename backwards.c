/*
 *
 * backwards.c
 *
 * A modification of neurotic.c that doubles the CPG in order to enable
 * the robot to walk backwards.
 *
 */

#include "libneurobot.h"

#include "backwards.h"

/* The strength of position feedback in pA. */
#define DEFAULT_FEEDBACK 25

/* The default time for the CPG to switch direction, in ms. */
#define DEFAULT_REVERSAL_TIME 10e3


int main(int argc, char**argv) 
{

    /*
     * Parsing command-line options.
     */
    float feedback = DEFAULT_FEEDBACK;
    float reverse_time_ms = DEFAULT_REVERSAL_TIME;

    int opt;
    char *endptr;
    while ((opt = getopt(argc, argv, "p:k:r:")) != -1) {
        if (opt == 'p') {
            set_pwm_max(strtod(optarg, &endptr));
            if (endptr && *endptr != '\0')
                die("Invalid PWM maximum", optarg);
        } else if (opt == 'k') {
            feedback = strtod(optarg, &endptr);
            if (endptr && *endptr != '\0') 
                die("Invalid feedback constant", optarg);
        } else if (opt == 'r') {
            reverse_time_ms = strtod(optarg, &endptr)*1000;
            if (endptr && *endptr != '\0')
                die("Invalid reversal time", optarg);
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

    bool reversed_yet = false;
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

                /* Swap prev and next for the reverse network. */
                if (i >= 12) {
                    int tmp = prev;
                    prev = next;
                    next = tmp;
                }
                
                float prev_err = fabs(1 - actuator_position[prev]);
                float next_err = fabs(0 - actuator_position[next]);
                i_in += -feedback*(prev_err + next_err);
            }

            if (!reversed_yet && get_current_time() >= reverse_time_ms) {
                printf("Hit %f s, reversing.\n", get_current_time()/1e3);

                /* 
                 * Reversing is accomplished by causing all the
                 * inhibitory cells in the forward CPG to spike,
                 * guaranteeing that it stops, and causing an arbitrary
                 * cell in the reverse CPG to spike so it starts. This
                 * is faked by setting the presynaptic activation
                 * derivative j to 1 the same way a spike does.
                 */
                states[2].j = states[5].j = states[8].j = states[11].j = 1;
                states[12].j = 1;

                reversed_yet = true;
            }

            resolve_dynamics(&states[i], params[i], i_in);

            float vlog = states[i].v;
            if (vlog > params[i]->vp) vlog = params[i]->vp;
            datalogf(", %f", vlog);
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

