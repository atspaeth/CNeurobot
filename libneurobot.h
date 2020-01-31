
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


struct state {
    float v, u, i, j;
};

struct params {
    float C, k, tau;
    float a, b, c, d;
    float vr, vt, vp;
};

/* 
 * Designated initializers don't support weird ranges, and I want the
 * parameters to be constant, so do something really freaky: define the
 * parameters for the two types as constant structs.
 */
#define RS {.a=0.03,.b=-2,.c=-50,.d=100,.C=100,.k=0.7,.vr=-60,.vt=-40,.vp=25,.tau=5}
#define LTS {.a=0.03,.b=8,.c=-53,.d=20,.C=100,.k=1,.vr=-56,.vt=-42,.vp=25,.tau=20}



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

int parse_common_option(int opt, const char *optarg);

float get_current_time();

void synchronize_loop();

void print_final_time();
