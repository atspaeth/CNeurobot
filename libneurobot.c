/*
 *
 * libneurobot.c
 *
 * The parts that ought to be the same between different Neurobot
 * programs, such as interfacing with the IO subsystem and running the
 * neuron dynamics. We also use the same file as the basis of the Python
 * neurobot module thanks to cffi. :)
 */

#include <libpruio/pruio.h>
#include <libpruio/pruio_pins.h>

#include "libneurobot.h"


/* Constant PWM frequency. */
#define PWM_FREQ_HZ 200.f




FILE *g_logfile = NULL;
pruIo *g_pru = NULL;
uint8_t g_pinmodes[4] = {};

const int pwm_pins[4] = {
    P9_31, P9_29, P9_14, P9_16
};

const int gpio_pins[4] = {
    P8_07, P8_08, P8_10, P8_09
};


void state_update(float dt, float i_in, 
        const struct state *st, const struct params *pr,
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

/* 
 * High-resolution timers to ensure that the loop
 * executes in real time according to the timestep.
 */
long g_num_dts = 0;
struct timespec g_start_time, g_last_time;

float get_current_time()
{
    return g_num_dts * dt_ms();
}

void setup()
{
    /* Set up the clocks. */
    clock_gettime(CLOCK_MONOTONIC, &g_last_time);
    g_start_time = g_last_time;

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
     * motors' enable lines. These begin at 0% duty cycle.
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
}


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

int datalogf(const char *fmt, ...) 
{
    if (g_logfile == NULL) return 0;

    va_list args;
    va_start(args, fmt);
    int ret = vfprintf(g_logfile, fmt, args);
    va_end(args);
    return ret;
}



/*
 * Store the pointer to the logfile object for debug logging.
 */
void open_logfile(const char *path) 
{
    if (!strncmp("-", path, 2)) g_logfile = stdout;
    else g_logfile = fopen(path, "w");

    if (g_logfile == NULL) {
        perror("Couldn't open logfile");
        exit(1);
    }
}


/*
 * Read the ADC value, taking a 12-bit ADC value and converting it to a
 * floating-point number in the interval [0,1]. 
 */
float read_adc(int i)
{
    uint16_t raw = g_pru->Adc->Value[i+1];
    return 1.f*raw / (1<<12);
}


/*
 * Check if a cell should spike, and if so, update its state to
 * represent the fact that this has happened. Also return whether it did
 * in case someone wants to use that fact.
 */
bool check_spike(struct state *state, const struct params *params) 
{
    if (state->v < params->vp) 
        return false;

    state->v = params->c; 
    state->u += params->d;
    state->j += 1;
    return true;
}


/*
 * Midpoint-method integration of the cell dynamics.  There's no
 * particular reason for the choice of integration method besides that
 * it's not much more work than forward Euler but second-order
 * correctness probably helps.
 */
void resolve_dynamics(struct state *state, const struct params *param, 
        float i_in)
{
    struct state tmpstate = *state;
    state_update(dt_ms()/2, i_in, state, param, &tmpstate);
    state_update(dt_ms(), i_in, &tmpstate, param, state);
}




/* 
 * Safety feature: max PWM duty cycle. 
 */
#define DEFAULT_PWM_MAX 0.3
float g_pwm_max = DEFAULT_PWM_MAX;

/* 
 * Apply an activation effort to the ith actuator, using the sign to set
 * the direction pin and the magnitude to calculate the duty cycle for
 * the enable pin (as a fraction of the allowed maximum).
 */
void apply_actuator(size_t i, float activation) 
{
    if (activation > 1) activation = 1;
    if (activation < -1) activation = -1;
    float duty_cycle = activation * g_pwm_max;

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


/*
 * Set the maximum PWM percentage, rather than trying to do something
 * weird with parsing options. :)
 */
void set_pwm_max(float percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    g_pwm_max = percent/100;
}


/* The simulation timestep. */
int g_dt_us = 500;

float dt_ms() {
    return (float)g_dt_us / US_PER_MS;
}


static long g_total_sleep_us = 0;


/* 
 * Check how long it has been, then sleep for the rest of the timestep.
 * Then update the official start time of the timestep such that we'll
 * correct for "oversleeping".
 */
void synchronize_loop()
{
    struct timespec this_time;

    clock_gettime(CLOCK_MONOTONIC, &this_time);
    long delta_t_us = 
        (this_time.tv_nsec - g_last_time.tv_nsec) / NS_PER_US
        + (this_time.tv_sec - g_last_time.tv_sec) * US_PER_SEC;

    g_last_time.tv_nsec += g_dt_us * NS_PER_US;
    if (g_last_time.tv_nsec > NS_PER_SEC) {
        g_last_time.tv_sec += 1;
        g_last_time.tv_nsec -= NS_PER_SEC;
    }

    if (delta_t_us < g_dt_us) {
        g_total_sleep_us += g_dt_us - delta_t_us;
        usleep(g_dt_us - delta_t_us);
    } 

    g_num_dts++;
}


void print_final_time()
{
    struct timespec stop_time;
    clock_gettime(CLOCK_MONOTONIC, &stop_time);
    long delta_t_us = 
        (stop_time.tv_nsec - g_start_time.tv_nsec) / NS_PER_US
        + (stop_time.tv_sec - g_start_time.tv_sec) * US_PER_SEC;
    fprintf(stderr, "Simulated %ld steps in %ldms.\n", 
            g_num_dts, delta_t_us / 1000);
    fprintf(stderr, " (Timestep %ldμs actual, %dμs nominal.)\n",
            delta_t_us / g_num_dts, g_dt_us);
    fprintf(stderr, " (Slept on average %ldμs per step.)\n",
            g_total_sleep_us / g_num_dts);
}



