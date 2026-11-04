#include "main.h"

// ---------------------------------------------------------------------------
// Physical Port Layout Mapping (Inverted Left-to-Right & Top-to-Bottom)
// ---------------------------------------------------------------------------
constexpr int TOP_LEFT_PORTS[]   = {20, 19, 18, 17, 16};
constexpr int TOP_RIGHT_PORTS[]  = {15, 14, 13, 12, 11};

constexpr int BOTTOM_LEFT_PORTS[]  = {10, 9, 8, 7, 6};
constexpr int BOTTOM_RIGHT_PORTS[] = {5, 4, 3, 2, 1};

constexpr int SIDE_PORT = 21;

// ---------------------------------------------------------------------------
// Optional Motor Labels
// ---------------------------------------------------------------------------
struct Label { int port; const char* text; };
constexpr Label LABELS[] = {
    // {1,  "LF"},
    // {2,  "LB"},
    // {11, "RF"},
    // {12, "RB"},
    // {5,  "ARM"},
    {0, ""}   // End marker
};

constexpr bool SHOW_PORT_NUMBERS = false;
constexpr bool SHOW_RULER        = false;

constexpr int LABEL_COUNT = sizeof(LABELS) / sizeof(LABELS[0]);

// Layout offsets (pixels)
constexpr int LEFT_X0     = 3;    // Left block starting position
constexpr int RIGHT_X0    = 265;  // Shifted right to double the center gap
constexpr int COL_W       = 43;   // Spacing between box centers
constexpr int BOX_W       = 39;   
constexpr int BOX_H       = 24;   
constexpr int EDGE_GAP    = 2;    

constexpr int SCREEN_W = 480;
constexpr int SCREEN_H = 240;

constexpr int SIDE_X = EDGE_GAP;
constexpr int SIDE_Y = (SCREEN_H - BOX_H) / 2;

// Temperature Thresholds (Celsius)
constexpr double TEMP_WARM = 45.0;
constexpr double TEMP_HOT  = 55.0;

// Colors
constexpr uint32_t COLOR_BG     = 0x000000;
constexpr uint32_t COLOR_GREEN  = 0x00A000; // Normal (< 45C)
constexpr uint32_t COLOR_ORANGE = 0xE08000; // Warm (45C - 54C)
constexpr uint32_t COLOR_RED    = 0xD00000; // Hot (>= 55C)
constexpr uint32_t COLOR_EMPTY  = 0x303030; // Unplugged / Non-motor outline
constexpr uint32_t COLOR_BORDER = 0xFFFFFF;
constexpr uint32_t COLOR_TEXT   = 0xFFFFFF;
constexpr uint32_t COLOR_LABEL  = 0xC0C0C0;

uint32_t tempColor(double c) {
    if (c >= TEMP_HOT)  return COLOR_RED;
    if (c >= TEMP_WARM) return COLOR_ORANGE;
    return COLOR_GREEN;
}

const char* labelFor(int port) {
    for (int i = 0; i < LABEL_COUNT; i++) {
        if (LABELS[i].port == port && LABELS[i].text[0] != '\0') {
            return LABELS[i].text;
        }
    }
    return nullptr;
}

void drawBox(int port, int x1, int y1, int labelX, int labelY) {
    int x2 = x1 + BOX_W;
    int y2 = y1 + BOX_H;
    int cx = x1 + BOX_W / 2;
    int cy = y1 + BOX_H / 2;

    double t = pros::c::motor_get_temperature(port);

    bool hasMotor = (t != PROS_ERR_F && t > 0.0);

    if (hasMotor) {
        pros::screen::set_pen(tempColor(t));
        pros::screen::fill_rect(x1, y1, x2, y2);
        pros::screen::set_pen(COLOR_BORDER);
    } else {
        pros::screen::set_pen(COLOR_EMPTY);
    }
    pros::screen::draw_rect(x1, y1, x2, y2);

    if (SHOW_PORT_NUMBERS) {
        pros::screen::set_pen(COLOR_TEXT);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, cx, cy, "%d", port);
    } else if (hasMotor) {
        pros::screen::set_pen(COLOR_TEXT);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, cx, cy, "%.0f", t);
    }

    const char* label = labelFor(port);
    if (label != nullptr && !SHOW_PORT_NUMBERS) {
        pros::screen::set_pen(COLOR_LABEL);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, labelX, labelY, "%.4s", label);
    }
}

void drawRuler() {
    pros::screen::set_pen(COLOR_TEXT);
    for (int x = 0; x <= SCREEN_W; x += 10) {
        int len = (x % 50 == 0) ? 14 : 7;
        pros::screen::draw_line(x, 0, x, len);
        pros::screen::draw_line(x, SCREEN_H - 1, x, SCREEN_H - 1 - len);
        if (x % 50 == 0 && x > 0 && x < SCREEN_W) {
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, x, 28, "%d", x);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, x, SCREEN_H - 28, "%d", x);
        }
    }
}

void tempScreenTask(void*) {
    while (true) {
        pros::screen::set_eraser(COLOR_BG);
        pros::screen::erase();

        if (SHOW_RULER) {
            drawRuler();
            pros::delay(250);
            continue;
        }

        int topY    = EDGE_GAP;
        int bottomY = SCREEN_H - EDGE_GAP - BOX_H;

        // Top Row Left Group: Ports 20 to 16
        for (int i = 0; i < 5; i++) {
            int x = LEFT_X0 + i * COL_W;
            drawBox(TOP_LEFT_PORTS[i], x, topY, x + BOX_W / 2, topY + BOX_H + 12);
        }

        // Top Row Right Group: Ports 15 to 11
        for (int i = 0; i < 5; i++) {
            int x = RIGHT_X0 + i * COL_W;
            drawBox(TOP_RIGHT_PORTS[i], x, topY, x + BOX_W / 2, topY + BOX_H + 12);
        }

        // Bottom Row Left Group: Ports 10 to 6
        for (int i = 0; i < 5; i++) {
            int x = LEFT_X0 + i * COL_W;
            drawBox(BOTTOM_LEFT_PORTS[i], x, bottomY, x + BOX_W / 2, bottomY - 12);
        }

        // Bottom Row Right Group: Ports 5 to 1
        for (int i = 0; i < 5; i++) {
            int x = RIGHT_X0 + i * COL_W;
            drawBox(BOTTOM_RIGHT_PORTS[i], x, bottomY, x + BOX_W / 2, bottomY - 12);
        }

        // Side Port (Port 21)
        drawBox(SIDE_PORT, SIDE_X, SIDE_Y, SIDE_X + BOX_W + 12, SIDE_Y + BOX_H / 2);

        pros::delay(250);
    }
}

void initialize() {
    pros::Task screenTask(tempScreenTask, nullptr, "temp screen");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}
void opcontrol() {}