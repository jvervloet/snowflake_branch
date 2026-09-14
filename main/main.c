#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_random.h"
#include "led_strip.h"

#define LEDS_PER_STRIP        25
#define LED_STRIP_FRAME_MS    20

// Push button: active-low (internal pull-up), cycles through animation modes.
#define BUTTON_GPIO           18
#define BUTTON_DEBOUNCE_MS    50

// The RMT peripheral can only drive SOC_RMT_TX_CANDIDATES_PER_GROUP (4 on
// ESP32-S3) channels at once, so the remaining 2 branches are bit-banged
// over the two general-purpose SPI hosts (SPI2/SPI3) instead.
#define NUM_RMT_STRIPS  4
#define NUM_SPI_STRIPS  2
#define NUM_STRIPS      (NUM_RMT_STRIPS + NUM_SPI_STRIPS)
#define TOTAL_LEDS      (NUM_STRIPS * LEDS_PER_STRIP)

static const int strip_gpio[NUM_STRIPS] = {4, 5, 6, 7, 15, 16};
static const spi_host_device_t spi_hosts[NUM_SPI_STRIPS] = {SPI2_HOST, SPI3_HOST};

// --- Layout, for modes that address LEDs by (x, y) position ---
//
// Only the shape matters here, not any real-world size -- these coordinates
// are unitless, just proportioned to match the reference diagram. +y points
// outward from the snowflake center and +x is to the right when facing
// outward. This mirrors the wiring order (LED numbers as given, 1-based):
//   LED  1-5:  straight out
//   LED  6-9:  branch left
//   LED 10-13: branch right
//   LED 14-16: straight out
//   LED 17-19: branch left
//   LED 20-22: branch right
//   LED 23-25: straight out, to the tip
static const float branch_led_pos[LEDS_PER_STRIP][2] = {
    // LED 1-5: straight out
    {0, 20}, {0, 45}, {0, 70}, {0, 95}, {0, 120},
    // LED 6-9: branch left
    {-20, 135}, {-40, 150}, {-60, 165}, {-80, 180},
    // LED 10-13: branch right
    {20, 135}, {40, 150}, {60, 165}, {80, 180},
    // LED 14-16: straight out
    {0, 155}, {0, 190}, {0, 225},
    // LED 17-19: branch left
    {-25, 240}, {-50, 255}, {-75, 270},
    // LED 20-22: branch right
    {25, 240}, {50, 255}, {75, 270},
    // LED 23-25: straight out, to the tip
    {0, 260}, {0, 300}, {0, 340},
};

// Outward-facing angle of each branch (degrees, standard math convention:
// 0 = +X/right, 90 = +Y/up, CCW positive), in strip_gpio[] order. Confirmed
// against the physical build with ANIM_CHANNEL_TEST: strip 0 (GPIO4) is the
// top branch, and each next strip is 60 degrees counterclockwise from the
// last (branch 2 is to the left of branch 1).
static const float branch_angle_deg[NUM_STRIPS] = {90, 150, -150, -90, -30, 30};

static float led_pos[TOTAL_LEDS][2];

// --- Image grid, for modes that display a 2D picture ---
//
// A GRID_SIZE x GRID_SIZE grid is overlaid on the whole snowflake, spanning
// [-grid_extent, +grid_extent] on both axes (grid_extent is the distance to
// the single farthest LED, so the grid always just covers the full layout,
// whatever unit branch_led_pos[] happens to be in). Every LED is assigned to
// whichever cell its (x, y) position falls into. An "image" is then just a
// GRID_SIZE x GRID_SIZE array of colors -- render_bitmap() below reads each
// LED's color from its cell.
//
// With only 150 LEDs total, most cells map to 0 or 1 LEDs -- the LEDs
// themselves are the real resolution limit, not this grid size.
#define GRID_SIZE 40

typedef struct {
    uint8_t r, g, b;
} rgb_t;

static uint8_t led_grid_col[TOTAL_LEDS];
static uint8_t led_grid_row[TOTAL_LEDS]; // row 0 = top (max y), increasing downward
static float grid_extent;
static float led_radius[TOTAL_LEDS]; // each LED's distance from the snowflake center

static void build_led_positions(void)
{
    for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
        // Rotate the canonical (outward = +y) branch template so it points
        // along this branch's actual angle.
        float delta = (branch_angle_deg[strip_idx] - 90.0f) * (float)M_PI / 180.0f;
        float c = cosf(delta);
        float s = sinf(delta);
        for (int pixel = 0; pixel < LEDS_PER_STRIP; pixel++) {
            float x = branch_led_pos[pixel][0];
            float y = branch_led_pos[pixel][1];
            int idx = strip_idx * LEDS_PER_STRIP + pixel;
            led_pos[idx][0] = x * c - y * s;
            led_pos[idx][1] = x * s + y * c;
        }
    }

    grid_extent = 0.0f;
    for (int i = 0; i < TOTAL_LEDS; i++) {
        led_radius[i] = sqrtf(led_pos[i][0] * led_pos[i][0] + led_pos[i][1] * led_pos[i][1]);
        if (led_radius[i] > grid_extent) {
            grid_extent = led_radius[i];
        }
    }

    float cell_size = (2.0f * grid_extent) / GRID_SIZE;
    for (int i = 0; i < TOTAL_LEDS; i++) {
        int col = (int)((led_pos[i][0] + grid_extent) / cell_size);
        int row = (int)((grid_extent - led_pos[i][1]) / cell_size);
        if (col < 0) col = 0;
        if (col >= GRID_SIZE) col = GRID_SIZE - 1;
        if (row < 0) row = 0;
        if (row >= GRID_SIZE) row = GRID_SIZE - 1;
        led_grid_col[i] = (uint8_t)col;
        led_grid_row[i] = (uint8_t)row;
    }
}

// Writes each LED's color from whichever grid cell it falls into.
static void render_bitmap(led_strip_handle_t strips[NUM_STRIPS], const rgb_t bitmap[GRID_SIZE][GRID_SIZE])
{
    for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
        for (int pixel = 0; pixel < LEDS_PER_STRIP; pixel++) {
            int idx = strip_idx * LEDS_PER_STRIP + pixel;
            rgb_t color = bitmap[led_grid_row[idx]][led_grid_col[idx]];
            led_strip_set_pixel(strips[strip_idx], pixel, color.r, color.g, color.b);
        }
    }
}

// Example image: a square drawn green, sized relative to the largest square
// that fits inside the circle traced by the farthest LED (half-side =
// radius / sqrt(2)). Swap this out (or add more build_..._bitmap()
// functions) for other test images.
#define BITMAP_SQUARE_BRIGHTNESS  0.5f
#define BITMAP_SQUARE_SCALE       0.8f // fraction of the largest square that still fits

static void build_square_bitmap(rgb_t bitmap[GRID_SIZE][GRID_SIZE])
{
    float half_size = grid_extent * 0.70710678f * BITMAP_SQUARE_SCALE;
    float cell_size = (2.0f * grid_extent) / GRID_SIZE;
    uint8_t green = (uint8_t)(BITMAP_SQUARE_BRIGHTNESS * 255.0f + 0.5f);

    for (int row = 0; row < GRID_SIZE; row++) {
        float cell_y = grid_extent - (row + 0.5f) * cell_size;
        for (int col = 0; col < GRID_SIZE; col++) {
            float cell_x = -grid_extent + (col + 0.5f) * cell_size;
            bool inside_square = fabsf(cell_x) <= half_size && fabsf(cell_y) <= half_size;
            bitmap[row][col] = inside_square ? (rgb_t){0, green, 0} : (rgb_t){0, 0, 0};
        }
    }
}

// Snowfall: sparse white "drops" fall straight down the grid from row 0
// (top) to GRID_SIZE-1 (bottom), each with a short fading trail behind it.
// Drops are a few columns wide (rather than a single column) so there's
// always something lit somewhere in that band as it falls.
#define SNOWFALL_MAX_DROPS        12
#define SNOWFALL_TRAIL_LEN        5
#define SNOWFALL_MIN_WIDTH        2
#define SNOWFALL_MAX_WIDTH        3
#define SNOWFALL_FALL_MS          3000.0f // time for a drop to cross the whole grid
#define SNOWFALL_SPAWN_MIN_MS     120
#define SNOWFALL_SPAWN_MAX_MS     400
#define SNOWFALL_HEAD_BRIGHTNESS  0.9f

typedef struct {
    bool active;
    uint8_t col;    // leftmost column
    uint8_t width;  // SNOWFALL_MIN_WIDTH..SNOWFALL_MAX_WIDTH columns wide
    float row_pos;  // fractional row, increases as the drop falls
} snow_drop_t;

// Renders every active drop's head + trail into the grid; everything else
// stays off. Two overlapping drops in the same cell take the brighter one.
static void build_snowfall_bitmap(rgb_t bitmap[GRID_SIZE][GRID_SIZE], const snow_drop_t drops[SNOWFALL_MAX_DROPS])
{
    for (int row = 0; row < GRID_SIZE; row++) {
        for (int col = 0; col < GRID_SIZE; col++) {
            bitmap[row][col] = (rgb_t){0, 0, 0};
        }
    }

    uint8_t head_val = (uint8_t)(SNOWFALL_HEAD_BRIGHTNESS * 255.0f + 0.5f);
    for (int d = 0; d < SNOWFALL_MAX_DROPS; d++) {
        if (!drops[d].active) {
            continue;
        }
        int head_row = (int)floorf(drops[d].row_pos);
        for (int k = 0; k < SNOWFALL_TRAIL_LEN; k++) {
            int row = head_row - k;
            if (row < 0 || row >= GRID_SIZE) {
                continue;
            }
            uint8_t val = (uint8_t)(head_val * (1.0f - (float)k / SNOWFALL_TRAIL_LEN));
            for (int w = 0; w < drops[d].width; w++) {
                int col = drops[d].col + w;
                if (val > bitmap[row][col].r) {
                    bitmap[row][col] = (rgb_t){val, val, val};
                }
            }
        }
    }
}

// Shared slow "breathing" curve used by both fade animations.
#define BASE_BRIGHTNESS_MIN   0.10f
#define BASE_BRIGHTNESS_MAX   0.20f
#define BASE_FADE_PERIOD_MS   6000.0f

// Time for one full rotation of the red/blue split around the center.
#define RB_ROTATION_PERIOD_MS 2000.0f

// Random per-pixel sparkles layered on top of the white breathing base.
#define SPARKLE_PEAK_BRIGHTNESS  1.0f
#define SPARKLE_DECAY_MS         180
#define SPARKLE_MIN_GAP_MS       120
#define SPARKLE_MAX_GAP_MS       900
#define SPARKLE_MAX_CONCURRENT   8

typedef struct {
    bool active;
    int led_index; // flat index into [0, TOTAL_LEDS)
    int elapsed_ms;
} sparkle_t;

// Radial pulse: a few cool-white rings expand outward from the center to
// the branch tips and fade out, using each LED's distance from center
// (led_radius[]) directly -- no grid needed, since a ring test is just
// "how far is this LED's radius from the ring's current radius".
#define PULSE_MAX_CONCURRENT   3
#define PULSE_TRAVEL_MS        2200.0f // time for a ring to travel center -> farthest LED
#define PULSE_BAND_FRACTION    0.10f   // ring thickness, as a fraction of grid_extent
#define PULSE_SPAWN_MIN_MS     500
#define PULSE_SPAWN_MAX_MS     1400
#define PULSE_BRIGHTNESS       0.8f

typedef struct {
    bool active;
    float radius; // current ring radius, grows from 0 out past grid_extent
} pulse_t;

// Test pattern: branch (strip index) N lights its first N LEDs (counting
// from the center) and no others -- branch 1 shows 1 LED, branch 2 shows 2,
// ... branch 6 shows 6. Counting lit LEDs on a physical branch tells you
// which strip_gpio[] entry (and so which GPIO) actually drives it.
#define CHANNEL_TEST_BRIGHTNESS 0.6f

// Button cycles: white sparkle (default) -> red/blue fade -> snowfall ->
// radial pulse -> bitmap image -> channel test -> off -> ...
typedef enum {
    ANIM_WHITE_SPARKLE,
    ANIM_RED_BLUE_FADE,
    ANIM_SNOWFALL,
    ANIM_PULSE,
    ANIM_BITMAP,
    ANIM_CHANNEL_TEST,
    ANIM_OFF,
    ANIM_MODE_COUNT,
} anim_mode_t;

static void init_strips(led_strip_handle_t strips[NUM_STRIPS])
{
    for (int i = 0; i < NUM_RMT_STRIPS; i++) {
        led_strip_config_t strip_config = {
            .strip_gpio_num = strip_gpio[i],
            .max_leds = LEDS_PER_STRIP,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .led_model = LED_MODEL_WS2812,
            .flags.invert_out = false,
        };
        led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .flags.with_dma = false,
        };
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strips[i]));
    }

    for (int i = 0; i < NUM_SPI_STRIPS; i++) {
        int idx = NUM_RMT_STRIPS + i;
        led_strip_config_t strip_config = {
            .strip_gpio_num = strip_gpio[idx],
            .max_leds = LEDS_PER_STRIP,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .led_model = LED_MODEL_WS2812,
            .flags.invert_out = false,
        };
        led_strip_spi_config_t spi_config = {
            .clk_src = SPI_CLK_SRC_DEFAULT,
            .spi_bus = spi_hosts[i],
            .flags.with_dma = true,
        };
        ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &strips[idx]));
    }

    for (int i = 0; i < NUM_STRIPS; i++) {
        ESP_ERROR_CHECK(led_strip_clear(strips[i]));
    }
}

static void init_button(void)
{
    gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));
}

// Advances a phase accumulator by one frame around the shared breathing period.
static float advance_phase(float phase)
{
    phase += (2.0f * (float)M_PI) * LED_STRIP_FRAME_MS / BASE_FADE_PERIOD_MS;
    if (phase > 2.0f * (float)M_PI) {
        phase -= 2.0f * (float)M_PI;
    }
    return phase;
}

// Maps a phase to a brightness between BASE_BRIGHTNESS_MIN and BASE_BRIGHTNESS_MAX.
static float breathing_brightness(float phase)
{
    float t = (sinf(phase) + 1.0f) * 0.5f; // 0..1
    return BASE_BRIGHTNESS_MIN + t * (BASE_BRIGHTNESS_MAX - BASE_BRIGHTNESS_MIN);
}

void app_main(void)
{
    led_strip_handle_t strips[NUM_STRIPS];
    init_strips(strips);
    init_button();
    build_led_positions();

    static rgb_t square_bitmap[GRID_SIZE][GRID_SIZE];
    build_square_bitmap(square_bitmap);

    anim_mode_t mode = ANIM_WHITE_SPARKLE;
    int last_button_level = 1;
    int button_debounce_remaining_ms = 0;

    // White sparkle animation state.
    sparkle_t sparkles[SPARKLE_MAX_CONCURRENT] = {0};
    float white_fade_phase = 0.0f;
    int next_sparkle_in_ms = SPARKLE_MIN_GAP_MS + (esp_random() % (SPARKLE_MAX_GAP_MS - SPARKLE_MIN_GAP_MS));

    // Red/blue fade animation state.
    float rb_fade_phase = 0.0f;
    float rb_rotation_rad = 0.0f;

    // Snowfall animation state.
    static rgb_t snow_bitmap[GRID_SIZE][GRID_SIZE];
    snow_drop_t drops[SNOWFALL_MAX_DROPS] = {0};
    int next_drop_in_ms = SNOWFALL_SPAWN_MIN_MS + (esp_random() % (SNOWFALL_SPAWN_MAX_MS - SNOWFALL_SPAWN_MIN_MS));

    // Radial pulse animation state.
    pulse_t pulses[PULSE_MAX_CONCURRENT] = {0};
    int next_pulse_in_ms = PULSE_SPAWN_MIN_MS + (esp_random() % (PULSE_SPAWN_MAX_MS - PULSE_SPAWN_MIN_MS));

    while (1) {
        // Poll the button with a simple time-based debounce and cycle modes on
        // the high-to-low (press) edge.
        if (button_debounce_remaining_ms > 0) {
            button_debounce_remaining_ms -= LED_STRIP_FRAME_MS;
        } else {
            int level = gpio_get_level(BUTTON_GPIO);
            if (level == 0 && last_button_level == 1) {
                mode = (anim_mode_t)((mode + 1) % ANIM_MODE_COUNT);
                if (mode == ANIM_OFF) {
                    for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
                        led_strip_clear(strips[strip_idx]);
                    }
                }
                button_debounce_remaining_ms = BUTTON_DEBOUNCE_MS;
            }
            last_button_level = level;
        }

        if (mode == ANIM_OFF) {
            vTaskDelay(pdMS_TO_TICKS(LED_STRIP_FRAME_MS));
            continue;
        }

        if (mode == ANIM_WHITE_SPARKLE) {
            // Base brightness follows a slow sine wave between 10% and 20%.
            float base_brightness = breathing_brightness(white_fade_phase);
            white_fade_phase = advance_phase(white_fade_phase);

            float pixel_brightness[TOTAL_LEDS];
            for (int i = 0; i < TOTAL_LEDS; i++) {
                pixel_brightness[i] = base_brightness;
            }

            // Kick off a new sparkle on a random LED (any strip) at a random interval.
            next_sparkle_in_ms -= LED_STRIP_FRAME_MS;
            if (next_sparkle_in_ms <= 0) {
                for (int s = 0; s < SPARKLE_MAX_CONCURRENT; s++) {
                    if (!sparkles[s].active) {
                        sparkles[s].active = true;
                        sparkles[s].led_index = esp_random() % TOTAL_LEDS;
                        sparkles[s].elapsed_ms = 0;
                        break;
                    }
                }
                next_sparkle_in_ms = SPARKLE_MIN_GAP_MS + (esp_random() % (SPARKLE_MAX_GAP_MS - SPARKLE_MIN_GAP_MS));
            }

            // Advance active sparkles: a fast rise to peak brightness, then decay back to base.
            for (int s = 0; s < SPARKLE_MAX_CONCURRENT; s++) {
                if (!sparkles[s].active) {
                    continue;
                }
                float progress = (float)sparkles[s].elapsed_ms / SPARKLE_DECAY_MS;
                if (progress >= 1.0f) {
                    sparkles[s].active = false;
                    continue;
                }
                float decay = 1.0f - progress;
                float sparkle_brightness = base_brightness + (SPARKLE_PEAK_BRIGHTNESS - base_brightness) * decay;
                pixel_brightness[sparkles[s].led_index] = sparkle_brightness;
                sparkles[s].elapsed_ms += LED_STRIP_FRAME_MS;
            }

            for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
                for (int pixel = 0; pixel < LEDS_PER_STRIP; pixel++) {
                    float brightness = pixel_brightness[strip_idx * LEDS_PER_STRIP + pixel];
                    uint8_t val = (uint8_t)(brightness * 255.0f + 0.5f);
                    led_strip_set_pixel(strips[strip_idx], pixel, val, val, val);
                }
            }
        } else if (mode == ANIM_RED_BLUE_FADE) {
            float brightness = breathing_brightness(rb_fade_phase);
            rb_fade_phase = advance_phase(rb_fade_phase);
            uint8_t val = (uint8_t)(brightness * 255.0f + 0.5f);

            // A red/blue half-and-half split that spins around the center,
            // decided per LED from its actual position (led_pos[]) so a
            // branch's own twigs can briefly show both colors as the
            // boundary sweeps past. branch_angle_deg[] now reflects the
            // true (counterclockwise) physical layout, so each LED's own
            // atan2 angle is already correct and self-consistent -- no
            // mirroring needed to make the local sweep (within a branch)
            // agree with the global sweep (across all 6 branches).
            rb_rotation_rad += (2.0f * (float)M_PI) * LED_STRIP_FRAME_MS / RB_ROTATION_PERIOD_MS;
            if (rb_rotation_rad > 2.0f * (float)M_PI) {
                rb_rotation_rad -= 2.0f * (float)M_PI;
            }

            for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
                for (int pixel = 0; pixel < LEDS_PER_STRIP; pixel++) {
                    int idx = strip_idx * LEDS_PER_STRIP + pixel;
                    float color_angle_rad = atan2f(led_pos[idx][1], led_pos[idx][0]);

                    float relative_angle_rad = fmodf(color_angle_rad - rb_rotation_rad, 2.0f * (float)M_PI);
                    if (relative_angle_rad < 0.0f) {
                        relative_angle_rad += 2.0f * (float)M_PI;
                    }
                    bool is_blue = relative_angle_rad < (float)M_PI;
                    if (is_blue) {
                        led_strip_set_pixel(strips[strip_idx], pixel, 0, 0, val);
                    } else {
                        led_strip_set_pixel(strips[strip_idx], pixel, val, 0, 0);
                    }
                }
            }
        } else if (mode == ANIM_SNOWFALL) {
            // Spawn a new drop at a random column at a random interval.
            next_drop_in_ms -= LED_STRIP_FRAME_MS;
            if (next_drop_in_ms <= 0) {
                for (int d = 0; d < SNOWFALL_MAX_DROPS; d++) {
                    if (!drops[d].active) {
                        drops[d].active = true;
                        drops[d].width = (uint8_t)(SNOWFALL_MIN_WIDTH + (esp_random() % (SNOWFALL_MAX_WIDTH - SNOWFALL_MIN_WIDTH + 1)));
                        drops[d].col = (uint8_t)(esp_random() % (GRID_SIZE - drops[d].width + 1));
                        drops[d].row_pos = -(float)SNOWFALL_TRAIL_LEN; // start just above the top
                        break;
                    }
                }
                next_drop_in_ms = SNOWFALL_SPAWN_MIN_MS + (esp_random() % (SNOWFALL_SPAWN_MAX_MS - SNOWFALL_SPAWN_MIN_MS));
            }

            // Advance active drops; retire one once its trail has fully exited the bottom.
            float row_step = (GRID_SIZE / SNOWFALL_FALL_MS) * LED_STRIP_FRAME_MS;
            for (int d = 0; d < SNOWFALL_MAX_DROPS; d++) {
                if (!drops[d].active) {
                    continue;
                }
                drops[d].row_pos += row_step;
                if ((int)floorf(drops[d].row_pos) - (SNOWFALL_TRAIL_LEN - 1) >= GRID_SIZE) {
                    drops[d].active = false;
                }
            }

            build_snowfall_bitmap(snow_bitmap, drops);
            render_bitmap(strips, snow_bitmap);
        } else if (mode == ANIM_PULSE) {
            // Spawn a new ring at a random interval.
            next_pulse_in_ms -= LED_STRIP_FRAME_MS;
            if (next_pulse_in_ms <= 0) {
                for (int p = 0; p < PULSE_MAX_CONCURRENT; p++) {
                    if (!pulses[p].active) {
                        pulses[p].active = true;
                        pulses[p].radius = 0.0f;
                        break;
                    }
                }
                next_pulse_in_ms = PULSE_SPAWN_MIN_MS + (esp_random() % (PULSE_SPAWN_MAX_MS - PULSE_SPAWN_MIN_MS));
            }

            // Advance active rings; retire one once it's fully past the farthest LED.
            float radius_step = (grid_extent / PULSE_TRAVEL_MS) * LED_STRIP_FRAME_MS;
            float band_half_width = grid_extent * PULSE_BAND_FRACTION * 0.5f;
            for (int p = 0; p < PULSE_MAX_CONCURRENT; p++) {
                if (!pulses[p].active) {
                    continue;
                }
                pulses[p].radius += radius_step;
                if (pulses[p].radius - band_half_width > grid_extent) {
                    pulses[p].active = false;
                }
            }

            // Each LED's brightness is set by whichever ring's edge it's
            // closest to (a triangular falloff across the ring's thickness);
            // overlapping rings take the brighter result.
            for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
                for (int pixel = 0; pixel < LEDS_PER_STRIP; pixel++) {
                    int idx = strip_idx * LEDS_PER_STRIP + pixel;
                    float best = 0.0f;
                    for (int p = 0; p < PULSE_MAX_CONCURRENT; p++) {
                        if (!pulses[p].active) {
                            continue;
                        }
                        float dist = fabsf(led_radius[idx] - pulses[p].radius);
                        if (dist < band_half_width) {
                            float t = 1.0f - dist / band_half_width;
                            if (t > best) {
                                best = t;
                            }
                        }
                    }
                    uint8_t val = (uint8_t)(best * PULSE_BRIGHTNESS * 255.0f + 0.5f);
                    uint8_t r = (uint8_t)(val * 0.75f);
                    uint8_t g = (uint8_t)(val * 0.88f);
                    led_strip_set_pixel(strips[strip_idx], pixel, r, g, val);
                }
            }
        } else if (mode == ANIM_BITMAP) {
            render_bitmap(strips, square_bitmap);
        } else { // ANIM_CHANNEL_TEST
            uint8_t val = (uint8_t)(CHANNEL_TEST_BRIGHTNESS * 255.0f + 0.5f);
            for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
                int lit_count = strip_idx + 1;
                for (int pixel = 0; pixel < LEDS_PER_STRIP; pixel++) {
                    if (pixel < lit_count) {
                        led_strip_set_pixel(strips[strip_idx], pixel, val, val, val);
                    } else {
                        led_strip_set_pixel(strips[strip_idx], pixel, 0, 0, 0);
                    }
                }
            }
        }

        for (int strip_idx = 0; strip_idx < NUM_STRIPS; strip_idx++) {
            led_strip_refresh(strips[strip_idx]);
        }

        vTaskDelay(pdMS_TO_TICKS(LED_STRIP_FRAME_MS));
    }
}
