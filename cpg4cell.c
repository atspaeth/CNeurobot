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

/* The strength of position feedback in pA. */
#define DEFAULT_FEEDBACK 40

/* Comment out to skip measuring actual dt. */
#define MEASURE_TIME


/* Boilerplate: number of cells including muscles. */
#define N_CELLS 8

/* 
 * Safety feature: max PWM duty cycle. Can be changed by command-line
 * options, but should never be increased above 40%, which is safe
 * according to the datasheet in the current configuration, where each
 * actuator is only active half of the time.
 */
#define DEFAULT_PWM_MAX 0.2

/* Constant PWM frequency. */
#define PWM_FREQ_HZ 200.f

/* The simulation timestep. */
#define DT_US (US_PER_MS/2)
#define DT_MS ((float)DT_US / US_PER_MS)
#define LOOP_TIME_US (DT_US)

/* Time conversion constants. */
#define NS_PER_US 1000
#define US_PER_MS 1000
#define MS_PER_SEC 1000
#define US_PER_SEC (US_PER_MS * MS_PER_SEC)
#define NS_PER_SEC (NS_PER_US * US_PER_SEC)

const int pwm_pins[4] = {
    P9_31, P9_29, P9_14, P9_16
};

const int gpio_pins[4] = {
    P8_07, P8_08, P8_10, P8_09
};

struct state {
    float v, u, i, j;
} states[] = {
    [0 ... N_CELLS-1] = {.v=-60}
};

struct params {
    float C, k, tau;
    float a, b, c, d;
    float vr, vt, vp;
} params[] = {
    [0 ... N_CELLS-1] = {
        .C=100, .k=0.7, .tau=5, 
        .a=0.03, .b=-2, .c=-50, .d=100,
        .vr=-60, .vt=-40, .vp=30 
    }
};


const float S[N_CELLS][N_CELLS] = {
    { 5000,-5000, -900,  900},
    {  900, 5000,-5000, -900},
    { -900,  900, 5000,-5000},
    {-5000, -900,  900, 5000},
    {  200,    0, -200,    0},
    {    0,  200,    0, -200},
    { -200,    0,  200,    0},
    {    0,- 200,    0,  200}
};

void state_update(float dt, float i_in, 
        struct state *st, struct params *pr,
        struct state *out) {
    float i_na = pr->k * (st->v - pr->vr) * (st->v - pr->vt);
    out->v += dt * (i_na - st->u + st->i + i_in) / pr->C;
    out->u += dt * pr->a * (pr->b*(st->v - pr->vr) - st->u);
    out->i += dt * st->j / pr->tau;
    out->j += dt * -(st->i + 2*st->j) / pr->tau;
}

void die(const char *message, const char *error)
{
    if (error) fprintf(stderr, "%s: %s\n", message, error);
    else fprintf(stderr, "%s :(\n", message);
    exit(1);
}

FILE *g_logfile = NULL;
pruIo *g_pru = NULL;
uint8_t g_pinmodes[4] = {};
void cleanup() 
{
    if (g_logfile != NULL && g_logfile != stdin) fclose(g_logfile);

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

int logprintf(const char *fmt, ...) 
{
    if (g_logfile == NULL) return 0;

    va_list args;
    va_start(args, fmt);
    int ret = vfprintf(g_logfile, fmt, args);
    va_end(args);
    return ret;
}


int main(int argc, char**argv) 
{

    float pwm_max = DEFAULT_PWM_MAX;
    float feedback = DEFAULT_FEEDBACK;

    int opt;
    char *endptr;
    while ((opt = getopt(argc, argv, "p:k:")) != -1) {
        switch (opt) {
        case 'p':
            pwm_max = strtod(optarg, &endptr)/100;
            if (endptr && *endptr != '\0') 
                die("Invalid PWM limit", optarg);
            break;
        case 'k':
            feedback = strtod(optarg, &endptr);
            if (endptr && *endptr != '\0') 
                die("Invalid feedback constant", optarg);
            break;
        }
    }

    /* 
     * If there's one argument and it's a filename, write to it. 
     */
    if (optind+1 < argc) die("Too many arguments!", NULL);
    if (optind+1 == argc) {
        if (!strncmp("-", argv[optind], 2)) g_logfile = stdout;
        else g_logfile = fopen(argv[optind], "w");

        if (g_logfile == NULL) {
            perror("Couldn't open logfile");
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
     * Initialize the four PWM pins corresponding to the
     * motors' enable lines. These run at 20kHz to stay
     * outside of audio range, and begin at 0% duty cycle.
     */
    for (int i = 0; i < 4; i++) {
        if (pruio_pwm_setValue(g_pru, pwm_pins[i], PWM_FREQ_HZ, 0))
            die("Couldn't set PWM", g_pru->Errr);
    }

    /*
     * Send the config to the PRU. The parameters set the
     * driver to IO mode (i.e. sampling on demand), activate
     * the four ADC channels we're actually using, give zero
     * sampling frequency because that's not used in IO mode,
     * and say to return raw 12-bit values.
     */
    if (pruio_config(g_pru, 1, 0xF<<1, 0, 0)) 
        die("Config failed", g_pru->Errr);
    
    float actuator_position[4];

    states[0].v = 0;

    /* 
     * High-resolution timers to ensure that the loop
     * executes in real time according to the timestep.
     */
    long num_dts = 0;
    struct timespec last_time;
    clock_gettime(CLOCK_MONOTONIC, &last_time);
#ifdef MEASURE_TIME
    struct timespec start_time = last_time;
#endif

    logprintf("t,A0,A1,A2,A3,V0,V1,V2,V3,V4,V5,V6,V7\n");
    while (!g_please_die_kthxbai) {
        logprintf("%f", num_dts * DT_MS);

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
         * Update each cell individually.
         */
        for (int i = 0; i < N_CELLS; i++) {

            /*
             * Process spikes by resetting all cells which 
             * have exceeded their spike peaks, then starting
             * synaptic current in the postsynpatic cells.
             */
            if (states[i].v >= params[i].vp) {
                states[i].v = params[i].c;
                states[i].u += params[i].d;

                for (int j = 0; j < N_CELLS; j++) {
                    states[j].j += S[j][i] / params[j].tau;
                }
            }

            float i_in = 0;
            if (i < 4) {
                int prev = (i+3)%4;
                int next = (i+1)%4;
                float prev_err = 0.85 - actuator_position[prev];
                float next_err = 0.15 - actuator_position[next];
                i_in = -feedback*(prev_err*prev_err + next_err*next_err);
            }

            /*
             * Midpoint-method integration of the cell dynamics.
             * There's no particular reason for the choice of
             * integration method besides that it's not much more work
             * than forward Euler but second-order correctness probably
             * helps.
             */
            struct state tmpstate = states[i];
            state_update(DT_MS/2, i_in, &states[i], &params[i], &tmpstate);
            state_update(DT_MS, i_in, &tmpstate, &params[i], &states[i]);

            logprintf(", %f", states[i].v);
        }

        /* 
         * Calculate an actuation effort (hopefully proportional to 
         * force) for each motor based on the membrane voltage of
         * the corresponding muscle cell.
         */
        for (int i = 0; i < 4; i++) {
            int flexor = i + N_CELLS-4;
            int extensor = (i + 2)%4 + N_CELLS-4;
            float v_mem = states[flexor].v - states[extensor].v;
            float duty_cycle = tanh(v_mem) * pwm_max;

            /* 
             * Set the PWM duty cycle. The argument of -1 says to keep the
             * frequency the same.
             */
            if (pruio_pwm_setValue(g_pru, pwm_pins[i], -1, 
                        fabs(duty_cycle)))
                die("Couldn't set PWM A", g_pru->Errr);

            /*
             * Also set the direction of the motor based on the
             * sign of the actuation effort.
             */
            bool is_negative = signbit(duty_cycle);
            int mask = (is_negative?0:128) | g_pinmodes[i];
            if (pruio_gpio_setValue(g_pru, gpio_pins[i], mask))
                die("Couldn't do GPIO", g_pru->Errr);
        }
        logprintf("\n");

        /* 
         * Check how long it has been, then sleep for the rest of
         * the timestep. Then update the official start time of the
         * timestep such that we'll correct for "oversleeping".
         */
        struct timespec this_time;
        clock_gettime(CLOCK_MONOTONIC, &this_time);
        long delta_t_us = 
            (this_time.tv_nsec - last_time.tv_nsec) / NS_PER_US
            + (this_time.tv_sec - last_time.tv_sec) * US_PER_SEC;

        last_time.tv_nsec += LOOP_TIME_US * NS_PER_US;
        if (last_time.tv_nsec > NS_PER_SEC) {
            last_time.tv_sec += 1;
            last_time.tv_nsec -= NS_PER_SEC;
        }

        if (delta_t_us < LOOP_TIME_US) {
            usleep(LOOP_TIME_US - delta_t_us);
        } 

        num_dts++;
    }

#ifdef MEASURE_TIME
    struct timespec stop_time;
    clock_gettime(CLOCK_MONOTONIC, &stop_time);
    long delta_t_us = 
        (stop_time.tv_nsec - start_time.tv_nsec) / NS_PER_US
        + (stop_time.tv_sec - start_time.tv_sec) * US_PER_SEC;
    fprintf(stderr, "Simulated %ld steps in %ldms.\n", 
            num_dts, delta_t_us / 1000);
    fprintf(stderr, " (Timestep %ldus actual, %dus nominal.)\n",
            delta_t_us / num_dts, DT_US);
#endif

    cleanup();
}

