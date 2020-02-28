/*
 *
 * neurotic.c
 *
 * The basic forward-only controller which produces the results reported
 * in the RoboSoft paper. This has been split up now so that related
 * scripts don't duplicate a bunch of code to produce the same behavior.
 * Most of the actual work is done in libneurobot.c.
 *
 */

#include "libneurobot.h"

/* The strength of position feedback in pA. */
#define DEFAULT_FEEDBACK 25

/* Boilerplate: number of cells including muscles. */
#define N_CELLS 16

/* The array of cell states. */
struct state states[] = {
    [0 ... N_CELLS-1] = {.v=-60, .u=0, .i=0, .j=0}
};

/*
 * Initialize the neuron parameters using two standard cell types.
 */
const struct params RS = {
    .a=0.03, .b=-2, .c=-50, .d=100,
    .C=100, .k=0.7, .tau=5,
    .vr=-60, .vt=-40, .vp=25, .vn=0
};

const struct params LTS = {
    .a=0.03, .b=8, .c=-53, .d=20,
    .C=100, .k=1, .tau=20,
    .vr=-56, .vt=-42, .vp=25, .vn=-70
};

const struct params *params[N_CELLS] = {
    [0] = &RS, [1] = &RS, [2] = &LTS,
    [3] = &RS, [4] = &RS, [5] = &LTS,
    [6] = &RS, [7] = &RS, [8] = &LTS,
    [9] = &RS, [10] = &RS, [11] = &LTS,
    [12 ... 15] = &RS
};

const float G[16][16] = {
    [0][1] = 20.0,
    [0][2] = 40.0,
    [0][10] = 10.0,
    [1][0] = 20.0,
    [1][2] = 40.0,
    [2][0] = 3.0,
    [2][1] = 3.0,
    [2][3] = 8.0,
    [3][1] = 10.0,
    [3][4] = 20.0,
    [3][5] = 40.0,
    [4][3] = 20.0,
    [4][5] = 40.0,
    [5][3] = 3.0,
    [5][4] = 3.0,
    [5][6] = 8.0,
    [6][4] = 10.0,
    [6][7] = 20.0,
    [6][8] = 40.0,
    [7][6] = 20.0,
    [7][8] = 40.0,
    [8][6] = 3.0,
    [8][7] = 3.0,
    [8][9] = 8.0,
    [9][7] = 10.0,
    [9][10] = 20.0,
    [9][11] = 40.0,
    [10][9] = 20.0,
    [10][11] = 40.0,
    [11][0] = 8.0,
    [11][9] = 3.0,
    [11][10] = 3.0,
    [12][1] = 1.0,
    [13][4] = 1.0,
    [14][7] = 1.0,
    [15][10] = 1.0,
};



int main(int argc, char**argv) 
{

    /*
     * Allow user specification of the feedback constant 
     * and maximum PWM duty cycle.
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

