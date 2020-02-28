/*
 *
 * backwards.c
 *
 * A modification of neurotic.c that doubles the CPG in order to enable
 * the robot to walk backwards.
 *
 */

#include "libneurobot.h"

/* The strength of position feedback in pA. */
#define DEFAULT_FEEDBACK 5

/* The default time for the CPG to switch direction, in ms. */
#define DEFAULT_REVERSAL_TIME 10e3

/* Boilerplate: number of cells including muscles. */
#define N_CELLS 28


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
    [12] = &RS, [13] = &RS, [14] = &LTS,
    [15] = &RS, [16] = &RS, [17] = &LTS,
    [18] = &RS, [19] = &RS, [20] = &LTS,
    [21] = &RS, [22] = &RS, [23] = &LTS,
    [24 ... 27] = &RS
};


const float S[N_CELLS][N_CELLS] = {
 {    0, 1000,-1000,    0,    0,    0,    0,    0,    0,    0,  400,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 { 1000,    0,-1000,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {  100,  100,    0,  400,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,  400,    0,    0, 1000,-1000,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0, 1000,    0,-1000,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,  100,  100,    0,  400,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,  400,    0,    0, 1000,-1000,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0, 1000,    0,-1000,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,  100,  100,    0,  400,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,  400,    0,    0, 1000,-1000, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0, 1000,    0,-1000, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {  400,    0,    0,    0,    0,    0,    0,    0,    0,  100,  100,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0, 1000,-1000,    0,  400,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
   1000,    0,-1000,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
    100,  100,    0,    0,    0,    0,    0,    0,    0,  400,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0, 1000,-1000,    0,  400,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0, 1000,    0,-1000,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
    400,    0,    0,  100,  100,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0, 1000,-1000,    0,  400,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0, 1000,    0,-1000,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,  400,    0,    0,  100,  100,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,  400,    0,    0,    0,    0,    0,    0,    0,    0, 1000,-1000},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0, 1000,    0,-1000},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,  400,    0,    0,  100,  100,    0},
 {    0,   40,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
      0,   40,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,   40,    0,    0,    0,    0,    0,    0,    0, 
      0,    0,    0,    0,   40,    0,    0,    0,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,   40,    0,    0,    0,    0, 
      0,    0,    0,    0,    0,    0,    0,   40,    0,    0,    0,    0},
 {    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   40,    0, 
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   40,    0},
}; 

int main(int argc, char**argv) 
{

    /*
     * Allow user specification of the feedback constant and when to
     * reverse direction.
     */
    float feedback = DEFAULT_FEEDBACK;
    float reverse_time_ms = DEFAULT_REVERSAL_TIME;

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

            /* Compute synaptic current. */
            for (int j = 0; j < N_CELLS; j++) {
                i_in += S[i][j] * states[j].i;
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

