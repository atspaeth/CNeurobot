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

/* Time conversion constants. */
#define NS_PER_US 1000
#define US_PER_MS 1000
#define MS_PER_SEC 1000
#define US_PER_SEC (US_PER_MS * MS_PER_SEC)
#define NS_PER_SEC (NS_PER_US * US_PER_SEC)

/* The simulation timestep. */
extern int g_dt_us;
float dt_ms();

/* Each neuron's state variables. */
struct state {
    float v, u, i, j;
};

/* Parameters per cell type. */
struct params {
    float C, k, tau;
    float a, b, c, d;
    float vr, vt, vp, vn;
};


void state_update(float dt, float i_in, 
        const struct state *current_state, 
        const struct params *cell_params,
        struct state *output_state);

void die(const char *message, const char *error);

void setup();

void cleanup();

extern bool g_please_die_kthxbai;
void die_gracefully(int signal);

int datalogf(const char *fmt, ...);

void open_logfile(const char *path);

float read_adc(int channel_index);

bool check_spike(struct state *state, const struct params *params);

void resolve_dynamics(struct state *state, 
        const struct params *param, float i_in);

void apply_actuator(size_t i, float signed_fractional_activation);

void set_pwm_max(float percentage);

float get_current_time();

void synchronize_loop();

void print_final_time();
