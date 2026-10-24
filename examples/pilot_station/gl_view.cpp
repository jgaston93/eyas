#include "gl_view.hpp"

#include <GLFW/glfw3.h>

#include <stb_easy_font.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "eyas/types.hpp"

namespace station {

namespace {

constexpr double kDeg = 180.0 / std::numbers::pi;
constexpr double kFt = 1.0 / 0.3048;
constexpr double kKt = 3600.0 / 1852.0;

enum class Camera : std::uint8_t { chase, cockpit, observer };

// A block standing on the ground, for scale and parallax cues.
struct Block {
    double x, y;            // [m] NED centre
    double half_x, half_y;  // [m]
    double height;          // [m]
    float shade;            // grey level
};

// The world's fixed layout (a scene, not data): the pad at the origin, a
// 1000 m × 45 m runway 300 m east of it running north, and blocks scattered
// by a fixed-seed generator outside both.
std::vector<Block> make_blocks() {
    std::vector<Block> out;
    std::uint32_t s = 20261017U;
    const auto next = [&]() {
        s = s * 1664525U + 1013904223U;  // Numerical Recipes LCG
        return static_cast<double>(s >> 8) / static_cast<double>(1U << 24);
    };
    while (out.size() < 600) {
        Block b{};
        b.x = (next() - 0.5) * 5000.0;
        b.y = (next() - 0.5) * 5000.0;
        b.half_x = 2.0 + 5.0 * next();
        b.half_y = 2.0 + 5.0 * next();
        b.height = 3.0 + 17.0 * next() * next();
        b.shade = static_cast<float>(0.35 + 0.4 * next());
        const bool near_pad = std::hypot(b.x, b.y) < 80.0;
        const bool on_runway = b.y > 230.0 && b.y < 370.0 && b.x > -300.0 && b.x < 900.0;
        if (!near_pad && !on_runway) {
            out.push_back(b);
        }
    }
    // Four 40 m masts around the pad, for height judgement in a hover.
    for (const auto& [x, y] : {std::pair{120.0, 0.0}, {-120.0, 0.0}, {0.0, 120.0}, {0.0, -120.0}}) {
        out.push_back({x, y, 0.4, 0.4, 40.0, 0.9F});
    }
    return out;
}

double finite_or(double v, double fallback) {
    return std::isfinite(v) ? v : fallback;
}

}  // namespace

struct GlView::Impl {
    GLFWwindow* window = nullptr;
    InputSource source;
    int joystick = -1;
    std::string device;
    ViewCommands pending;
    Camera camera = Camera::chase;
    bool help = true;
    std::string screenshot;
    std::vector<Block> blocks = make_blocks();
    std::array<char, 200000> text_buf{};

    void find_device();
    void read_device(RawInput& raw);
    void read_mouse_keyboard(RawInput& raw) const;
    void draw_world(const HudInfo& hud, int w, int h);
    static void draw_aircraft(const eyas::AircraftState& s, double ground_z);
    void draw_hud(const HudInfo& hud, int w, int h);
    void text(double x, double y, double scale, const char* s, float r = 1.0F, float g = 1.0F,
              float b = 1.0F);
};

namespace {

void on_key(GLFWwindow* w, int key, int /*scancode*/, int action, int /*mods*/) {
    auto* impl = static_cast<GlView::Impl*>(glfwGetWindowUserPointer(w));
    if (impl == nullptr || action != GLFW_PRESS) {
        return;
    }
    ViewCommands& c = impl->pending;
    switch (key) {
        case GLFW_KEY_ESCAPE:
            c.quit = true;
            break;
        case GLFW_KEY_P:
            c.pause = !c.pause;
            break;
        case GLFW_KEY_R:
            c.restart = true;
            break;
        case GLFW_KEY_H:
            impl->help = !impl->help;
            break;
        case GLFW_KEY_V:
            if (impl->camera == Camera::chase) {
                impl->camera = Camera::cockpit;
            } else if (impl->camera == Camera::cockpit) {
                impl->camera = Camera::observer;
            } else {
                impl->camera = Camera::chase;
            }
            break;
        case GLFW_KEY_1:
        case GLFW_KEY_2:
        case GLFW_KEY_3:
        case GLFW_KEY_4:
            c.start = key - GLFW_KEY_1;
            break;
        default:
            break;
    }
}

}  // namespace

GlView::GlView(InputSource source, bool help) : impl_(std::make_unique<Impl>()) {
    impl_->source = source;
    impl_->help = help;
    if (glfwInit() != GLFW_TRUE) {
        throw std::runtime_error("GLFW could not start (no display?)");
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_SAMPLES, 4);
    impl_->window = glfwCreateWindow(1280, 800, "Eyas pilot station", nullptr, nullptr);
    if (impl_->window == nullptr) {
        glfwTerminate();
        throw std::runtime_error("cannot open an OpenGL 2.1 window");
    }
    glfwMakeContextCurrent(impl_->window);
    // The main loop paces the frames; vsync would add a second clock.
    glfwSwapInterval(0);
    glfwSetWindowUserPointer(impl_->window, impl_.get());
    glfwSetKeyCallback(impl_->window, on_key);
    impl_->find_device();
}

GlView::~GlView() {
    if (impl_->window != nullptr) {
        glfwDestroyWindow(impl_->window);
    }
    glfwTerminate();
}

std::string_view GlView::device() const noexcept {
    return impl_->device;
}

bool GlView::device_connected() const noexcept {
    return impl_->joystick >= 0;
}

void GlView::Impl::find_device() {
    joystick = -1;
    device.clear();
    if (source != InputSource::gamepad && source != InputSource::joystick) {
        return;
    }
    for (int j = GLFW_JOYSTICK_1; j <= GLFW_JOYSTICK_LAST; ++j) {
        const bool ok = source == InputSource::gamepad ? glfwJoystickIsGamepad(j) == GLFW_TRUE
                                                       : glfwJoystickPresent(j) == GLFW_TRUE;
        if (ok) {
            joystick = j;
            const char* n =
                source == InputSource::gamepad ? glfwGetGamepadName(j) : glfwGetJoystickName(j);
            device = n != nullptr ? n : "device";
            return;
        }
    }
    device = source == InputSource::gamepad ? "no gamepad connected" : "no joystick connected";
}

void GlView::Impl::read_device(RawInput& raw) {
    if (joystick < 0 || glfwJoystickPresent(joystick) != GLFW_TRUE) {
        find_device();
        if (joystick < 0) {
            raw = RawInput{};
            return;
        }
    }
    raw = RawInput{};
    if (source == InputSource::gamepad) {
        GLFWgamepadstate st{};
        if (glfwGetGamepadState(joystick, &st) == GLFW_TRUE) {
            for (int i = 0; i <= GLFW_GAMEPAD_AXIS_LAST && i < kAxesPerDevice; ++i) {
                raw.axes[static_cast<std::size_t>(i)] = static_cast<double>(st.axes[i]);
            }
            for (int i = 0; i <= GLFW_GAMEPAD_BUTTON_LAST && i < kButtonsPerDevice; ++i) {
                raw.buttons[static_cast<std::size_t>(i)] = st.buttons[i] == GLFW_PRESS;
            }
        }
        return;
    }
    int n = 0;
    const float* axes = glfwGetJoystickAxes(joystick, &n);
    for (int i = 0; axes != nullptr && i < n && i < kAxesPerDevice; ++i) {
        raw.axes[static_cast<std::size_t>(i)] = static_cast<double>(axes[i]);
    }
    const unsigned char* buttons = glfwGetJoystickButtons(joystick, &n);
    for (int i = 0; buttons != nullptr && i < n && i < kButtonsPerDevice; ++i) {
        raw.buttons[static_cast<std::size_t>(i)] = buttons[i] == GLFW_PRESS;
    }
}

// Axes 0, 1: the pointer from the window centre while the left button is
// held (the stick springs to centre when it's let go); 2: collective keys
// W (up) and S (down); 3: pedal keys Z (left) and X (right). Buttons 0: T or
// the right mouse button (trim), 1: B (brakes), 2: C (trim reset), 3: A
// (SAS), 4: Q (pitch bias actuator).
void GlView::Impl::read_mouse_keyboard(RawInput& raw) const {
    raw = RawInput{};
    const auto key = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        double x = 0.0;
        double y = 0.0;
        int w = 1;
        int h = 1;
        glfwGetCursorPos(window, &x, &y);
        glfwGetWindowSize(window, &w, &h);
        raw.axes[0] = std::clamp(2.0 * x / std::max(w, 1) - 1.0, -1.0, 1.0);
        raw.axes[1] = std::clamp(2.0 * y / std::max(h, 1) - 1.0, -1.0, 1.0);
    }
    raw.axes[2] = (key(GLFW_KEY_W) ? 1.0 : 0.0) - (key(GLFW_KEY_S) ? 1.0 : 0.0);
    raw.axes[3] = (key(GLFW_KEY_X) ? 1.0 : 0.0) - (key(GLFW_KEY_Z) ? 1.0 : 0.0);
    raw.buttons[0] =
        key(GLFW_KEY_T) || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    raw.buttons[1] = key(GLFW_KEY_B);
    raw.buttons[2] = key(GLFW_KEY_C);
    raw.buttons[3] = key(GLFW_KEY_A);
    raw.buttons[4] = key(GLFW_KEY_Q);
    raw.buttons[5] = key(GLFW_KEY_F);
}

ViewCommands GlView::poll(RawInput& raw) {
    glfwPollEvents();
    ViewCommands c = impl_->pending;
    impl_->pending = {};
    c.quit = c.quit || glfwWindowShouldClose(impl_->window) == GLFW_TRUE;
    switch (impl_->source) {
        case InputSource::gamepad:
        case InputSource::joystick:
            impl_->read_device(raw);
            break;
        case InputSource::mouse_keyboard:
            impl_->read_mouse_keyboard(raw);
            break;
        default:
            break;
    }
    return c;
}

void GlView::Impl::text(double x, double y, double scale, const char* s, float r, float g,
                        float b) {
    // stb_easy_font takes a non-const string and returns quads of 4 vertices
    // of 16 bytes (x, y, z float; RGBA bytes).
    std::array<char, 1024> copy{};
    (void)std::snprintf(copy.data(), copy.size(), "%s", s);
    const int quads = stb_easy_font_print(0.0F, 0.0F, copy.data(), nullptr, text_buf.data(),
                                          static_cast<int>(text_buf.size()));
    glPushMatrix();
    glTranslated(x, y, 0.0);
    glScaled(scale, scale, 1.0);
    glColor3f(r, g, b);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, text_buf.data());
    glDrawArrays(GL_QUADS, 0, quads * 4);
    glDisableClientState(GL_VERTEX_ARRAY);
    glPopMatrix();
}

namespace {

// Loads a view matrix for a camera at `eye` (NED) with forward f and down d
// (unit, orthogonal): eye coordinates are (right, up, −forward).
void load_view(const eyas::Vec3& eye, const eyas::Vec3& f, const eyas::Vec3& d) {
    const eyas::Vec3 r = d.cross(f);
    const eyas::Vec3 u = -d;
    const eyas::Vec3 b = -f;
    const std::array<double, 16> m{r.x(),       u.x(),       b.x(),       0.0,   r.y(), u.y(),
                                   b.y(),       0.0,         r.z(),       u.z(), b.z(), 0.0,
                                   -r.dot(eye), -u.dot(eye), -b.dot(eye), 1.0};
    glLoadMatrixd(m.data());
}

// Forward and down for a camera looking from `eye` at `target`, level horizon.
void look_at(const eyas::Vec3& eye, const eyas::Vec3& target) {
    eyas::Vec3 f = target - eye;
    if (f.norm() < 1e-6) {
        f = eyas::Vec3::UnitX();
    }
    f.normalize();
    eyas::Vec3 r = eyas::Vec3::UnitZ().cross(f);
    if (r.norm() < 1e-6) {
        r = eyas::Vec3::UnitY();
    }
    r.normalize();
    load_view(eye, f, f.cross(r));
}

void box(double x0, double x1, double y0, double y1, double z0, double z1, float shade) {
    // z down: z0 is the top. Sides darker than the top.
    glBegin(GL_QUADS);
    glColor3f(shade, shade, shade);
    glVertex3d(x0, y0, z0);
    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y1, z0);
    glVertex3d(x0, y1, z0);
    const float side = shade * 0.75F;
    glColor3f(side, side, side * 0.95F);
    glVertex3d(x0, y0, z0);
    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y0, z1);
    glVertex3d(x0, y0, z1);
    glVertex3d(x0, y1, z0);
    glVertex3d(x1, y1, z0);
    glVertex3d(x1, y1, z1);
    glVertex3d(x0, y1, z1);
    const float end = shade * 0.6F;
    glColor3f(end, end, end * 0.95F);
    glVertex3d(x0, y0, z0);
    glVertex3d(x0, y1, z0);
    glVertex3d(x0, y1, z1);
    glVertex3d(x0, y0, z1);
    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y1, z0);
    glVertex3d(x1, y1, z1);
    glVertex3d(x1, y0, z1);
    glEnd();
}

}  // namespace

void GlView::Impl::draw_world(const HudInfo& hud, int w, int h) {
    const eyas::AircraftState& s = *hud.state;
    // The ground plane in NED: z = −(ground height − origin altitude); the
    // NED origin's altitude is the altitude at pos_n = 0.
    const double origin_alt = s.altitude + s.pos_n.z();
    const double gz = -(hud.ground_height - origin_alt);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const double aspect = static_cast<double>(w) / std::max(h, 1);
    const double near = 0.5;
    const double top = near * std::tan(0.5 * 60.0 / kDeg);
    glFrustum(-top * aspect, top * aspect, -top, top, near, 20000.0);
    glMatrixMode(GL_MODELVIEW);

    const eyas::Vec3 cg = s.pos_n;
    eyas::Vec3 eye = cg;
    switch (camera) {
        case Camera::cockpit: {
            const Eigen::Matrix3d c_nb = s.q_nb.toRotationMatrix();
            load_view(cg, c_nb.col(0), c_nb.col(2));
            break;
        }
        case Camera::chase: {
            const eyas::Vec3 f(std::cos(s.heading), std::sin(s.heading), 0.0);
            eye = cg - 30.0 * f + eyas::Vec3(0.0, 0.0, -8.0);
            eye.z() = std::min(eye.z(), gz - 1.5);
            look_at(eye, cg);
            break;
        }
        case Camera::observer:
            eye = eyas::Vec3(-60.0, -25.0, gz - 1.7);
            look_at(eye, cg);
            break;
    }

    const std::array<float, 4> sky{0.55F, 0.70F, 0.88F, 1.0F};
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, 1500.0F);
    glFogf(GL_FOG_END, 15000.0F);
    glFogfv(GL_FOG_COLOR, sky.data());
    glEnable(GL_FOG);
    glEnable(GL_DEPTH_TEST);

    // Ground, pushed back so the markings on it win the depth test. Fog is
    // computed at the vertices, so the ground is tiled: 200 m tiles in the
    // 6 km square around the eye, 2 km tiles out to 20 km.
    const double ex = eye.x();
    const double ey = eye.y();
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0F, 2.0F);
    glBegin(GL_QUADS);
    glColor3f(0.36F, 0.50F, 0.30F);
    const auto tile = [&](double x, double y, double size) {
        glVertex3d(x, y, gz);
        glVertex3d(x + size, y, gz);
        glVertex3d(x + size, y + size, gz);
        glVertex3d(x, y + size, gz);
    };
    constexpr double kCoarse = 2000.0;
    constexpr double kFine = 200.0;
    const double cx0 = std::floor(ex / kCoarse) * kCoarse;  // the eye's coarse cell
    const double cy0 = std::floor(ey / kCoarse) * kCoarse;
    for (int i = -10; i <= 10; ++i) {
        for (int j = -10; j <= 10; ++j) {
            const double x = cx0 + i * kCoarse;
            const double y = cy0 + j * kCoarse;
            if (std::abs(i) <= 1 && std::abs(j) <= 1) {
                for (int k = 0; k < 10; ++k) {
                    for (int l = 0; l < 10; ++l) {
                        tile(x + k * kFine, y + l * kFine, kFine);
                    }
                }
            } else {
                tile(x, y, kCoarse);
            }
        }
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);

    // Markings drawn on the ground: the runway, then the pad.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0F, 1.0F);
    glBegin(GL_QUADS);
    glColor3f(0.30F, 0.30F, 0.32F);
    glVertex3d(-200.0, 277.5, gz);
    glVertex3d(800.0, 277.5, gz);
    glVertex3d(800.0, 322.5, gz);
    glVertex3d(-200.0, 322.5, gz);
    glColor3f(0.45F, 0.45F, 0.45F);
    for (int i = 0; i < 32; ++i) {  // the pad: a 15 m disc as a fan of quads
        const double a0 = 2.0 * std::numbers::pi * i / 32.0;
        const double a1 = 2.0 * std::numbers::pi * (i + 1) / 32.0;
        glVertex3d(0.0, 0.0, gz);
        glVertex3d(15.0 * std::cos(a0), 15.0 * std::sin(a0), gz);
        glVertex3d(15.0 * std::cos(a1), 15.0 * std::sin(a1), gz);
        glVertex3d(0.0, 0.0, gz);
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);

    // Grid: 10 m lines near the eye, 100 m lines further out (optic flow).
    glBegin(GL_LINES);
    const auto grid = [&](double step, double extent, float shade) {
        glColor3f(shade * 0.8F, shade, shade * 0.75F);
        const double x0 = std::floor((ex - extent) / step) * step;
        const double y0 = std::floor((ey - extent) / step) * step;
        const int n = static_cast<int>(2.0 * extent / step) + 1;
        for (int i = 0; i <= n; ++i) {
            glVertex3d(x0 + i * step, ey - extent, gz);
            glVertex3d(x0 + i * step, ey + extent, gz);
            glVertex3d(ex - extent, y0 + i * step, gz);
            glVertex3d(ex + extent, y0 + i * step, gz);
        }
    };
    grid(10.0, 400.0, 0.42F);
    grid(100.0, 4000.0, 0.30F);
    // Runway centreline dashes and the pad's H.
    glColor3f(0.95F, 0.95F, 0.95F);
    for (int i = 0; i < 25; ++i) {
        glVertex3d(-180.0 + 40.0 * i, 300.0, gz);
        glVertex3d(-160.0 + 40.0 * i, 300.0, gz);
    }
    glColor3f(1.0F, 1.0F, 1.0F);
    glVertex3d(-4.0, -3.0, gz);
    glVertex3d(4.0, -3.0, gz);
    glVertex3d(-4.0, 3.0, gz);
    glVertex3d(4.0, 3.0, gz);
    glVertex3d(0.0, -3.0, gz);
    glVertex3d(0.0, 3.0, gz);
    glEnd();

    for (const Block& b : blocks) {
        box(b.x - b.half_x, b.x + b.half_x, b.y - b.half_y, b.y + b.half_y, gz - b.height, gz,
            b.shade);
    }

    if (camera != Camera::cockpit) {
        draw_aircraft(s, gz);
    }
    glDisable(GL_FOG);
}

// The aircraft as a symbol (a cue for attitude and disc tilt, not aircraft
// geometry): a cabin, a tail boom and fin, and the rotor disc at a nominal
// 8 m radius tilted by the multiblade flapping, with ψ = 0 aft and ψ = 90°
// on the right (a CCW rotor, as the UH-60A's).
void GlView::Impl::draw_aircraft(const eyas::AircraftState& s, double ground_z) {
    const Eigen::Matrix3d c = s.q_nb.toRotationMatrix();
    const std::array<double, 16> m{c(0, 0),     c(1, 0),     c(2, 0),     0.0,     c(0, 1), c(1, 1),
                                   c(2, 1),     0.0,         c(0, 2),     c(1, 2), c(2, 2), 0.0,
                                   s.pos_n.x(), s.pos_n.y(), s.pos_n.z(), 1.0};
    glPushMatrix();
    glMultMatrixd(m.data());
    box(-3.0, 4.0, -1.1, 1.1, -1.2, 1.0, 0.25F);
    box(-11.0, -3.0, -0.35, 0.35, -0.9, -0.2, 0.22F);
    box(-11.5, -10.0, -0.15, 0.15, -3.0, -0.6, 0.22F);
    const double b0 = finite_or(s.coning, 0.0);
    const double b1c = finite_or(s.flap_1c, 0.0);
    const double b1s = finite_or(s.flap_1s, 0.0);
    constexpr double kR = 8.0;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(0.1F, 0.1F, 0.1F, 0.35F);
    glVertex3d(0.0, 0.0, -2.5);
    for (int i = 0; i <= 48; ++i) {
        const double psi = 2.0 * std::numbers::pi * i / 48.0;
        const double beta = b0 + b1c * std::cos(psi) + b1s * std::sin(psi);
        glVertex3d(-kR * std::cos(psi), kR * std::sin(psi), -2.5 - kR * beta);
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();

    // Its shadow straight below, for height over the ground.
    glBegin(GL_LINES);
    glColor3f(0.15F, 0.2F, 0.12F);
    glVertex3d(s.pos_n.x() - 6.0, s.pos_n.y(), ground_z);
    glVertex3d(s.pos_n.x() + 6.0, s.pos_n.y(), ground_z);
    glVertex3d(s.pos_n.x(), s.pos_n.y() - 6.0, ground_z);
    glVertex3d(s.pos_n.x(), s.pos_n.y() + 6.0, ground_z);
    glEnd();
}

void GlView::Impl::draw_hud(const HudInfo& hud, int w, int h) {
    const eyas::AircraftState& s = *hud.state;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, w, h, 0.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const auto panel = [](double x, double y, double pw, double ph) {
        glColor4f(0.0F, 0.0F, 0.0F, 0.45F);
        glBegin(GL_QUADS);
        glVertex2d(x, y);
        glVertex2d(x + pw, y);
        glVertex2d(x + pw, y + ph);
        glVertex2d(x, y + ph);
        glEnd();
    };
    std::array<char, 512> b{};
    const double height_agl = s.altitude - hud.ground_height;
    // Directions without a speed are noise (D-043): this display blanks the
    // track below 1 kt of ground speed, and the angle of attack and
    // sideslip below 5 kt of airspeed.
    std::array<char, 16> trk{};
    std::array<char, 16> aoa{};
    std::array<char, 16> beta{};
    const auto field = [](std::array<char, 16>& out, bool valid, double v) {
        if (valid && std::isfinite(v)) {
            (void)std::snprintf(out.data(), out.size(), "%6.1f", v);
        } else {
            (void)std::snprintf(out.data(), out.size(), "   ---");
        }
    };
    field(trk, s.ground_speed * kKt >= 1.0, s.track * kDeg);
    field(aoa, s.true_airspeed * kKt >= 5.0, s.angle_of_attack * kDeg);
    field(beta, s.true_airspeed * kKt >= 5.0, s.sideslip * kDeg);

    // Flight data, left.
    panel(10, 40, 250, 270);
    (void)std::snprintf(b.data(), b.size(),
                        "KIAS %6.1f\nKTAS %6.1f\nGS   %6.1f kt\nTRK  %s\nVS   %6.0f fpm\n"
                        "ALT  %6.0f ft MSL\nHGT  %6.1f ft AGL\nHDG  %6.1f\nROLL %6.1f\nPITCH%6.1f\n"
                        "BETA %s\nAOA  %s\nNZ   %6.2f g",
                        s.indicated_airspeed * kKt, s.true_airspeed * kKt, s.ground_speed * kKt,
                        trk.data(), s.vertical_speed * kFt * 60.0, s.altitude * kFt,
                        height_agl * kFt, s.heading * kDeg, s.roll * kDeg, s.pitch * kDeg,
                        beta.data(), aoa.data(), s.load_factor);
    text(20, 50, 1.6, b.data());

    // Rotor and gear, right.
    panel(w - 350, 40, 340, 250);
    const eyas::AfcsSwitches& af = hud.applied.afcs;
    const char* sas = "OFF";
    if (af.sas_digital && af.sas_analog) {
        sas = "on";
    } else if (af.sas_digital) {
        sas = "digital";
    } else if (af.sas_analog) {
        sas = "analog";
    }
    // Power levers, 0 idle stop to 100 fly, or OFF with the fuel shut off
    // (D-056); ">" the lever moves it, "*" it waits for the lever (pickup).
    std::array<std::array<char, 8>, 2> lever{};
    for (std::size_t e = 0; e < lever.size(); ++e) {
        const eyas::EngineControls& ec = hud.applied.engines[e];
        if (ec.fuel_on) {
            (void)std::snprintf(lever[e].data(), lever[e].size(), "%3.0f", 100.0 * ec.power_lever);
        } else {
            (void)std::snprintf(lever[e].data(), lever[e].size(), "OFF");
        }
    }
    (void)std::snprintf(
        b.data(), b.size(),
        "NR     %6.1f %%\nTHRUST %6.1f kN\nTORQUE %6.1f kN.m\nPOWER  %6.0f kW\n"
        "B0 %5.2f B1C %5.2f B1S %5.2f deg\nTR THR %6.2f kN\nWOW %s  BRAKES %s\n"
        "SAS %s  PBA %s  FPS %s%s\nPCL %s%s%s %s%s%s  REF %5.1f %%",
        s.rotor_speed_percent, s.main_rotor_thrust * 1e-3, s.main_rotor_torque * 1e-3,
        s.main_rotor_power * 1e-3, s.coning * kDeg, s.flap_1c * kDeg, s.flap_1s * kDeg,
        s.tail_rotor_thrust * 1e-3, s.weight_on_wheels ? "yes" : "no",
        hud.applied.brake_left > 0.5 ? "set" : "off", sas, af.pitch_bias ? "on" : "OFF",
        af.fps ? "on" : "OFF", af.fps_turn ? "  TURN" : "", hud.engine_selection != 2 ? ">" : " ",
        lever[0].data(), hud.power_lever_held[0] ? "*" : " ", hud.engine_selection != 1 ? ">" : " ",
        lever[1].data(), hud.power_lever_held[1] ? "*" : " ",
        100.0 * hud.applied.rotor_speed_reference);
    text(w - 340, 50, 1.6, b.data());

    // Status, top.
    panel(0, 0, w, 30);
    (void)std::snprintf(b.data(), b.size(),
                        "%.*s | start %.*s | t %.1f s | RT %.2f | advance %.0f us (worst %.0f) | "
                        "overruns %lld | FG %s%s | %s%s%s",
                        static_cast<int>(hud.source.size()), hud.source.data(),
                        static_cast<int>(hud.start.size()), hud.start.data(), s.sim_time,
                        hud.realtime_ratio, hud.advance_us, hud.worst_advance_us,
                        static_cast<long long>(hud.overruns), hud.fg_sending ? "out" : "-",
                        hud.fg_lines >= 0 ? (" in " + std::to_string(hud.fg_lines)).c_str() : "",
                        hud.paused ? "PAUSED " : "", hud.trim_held ? "TRIM " : "",
                        hud.recording_segment > 0 ? "REC" : "");
    text(10, 8, 1.4, b.data());
    if (hud.input_lost) {
        text(w / 2.0 - 280, 140, 3.0, "INPUT LOST - paused until it returns", 1.0F, 0.35F, 0.25F);
    } else if (hud.input_waiting) {
        text(w / 2.0 - 200, 140, 3.0, "WAITING FOR INPUT", 1.0F, 0.85F, 0.3F);
    }
    if (hud.crashed) {
        text(w / 2.0 - 250, 100, 3.0, "CRASHED - R or 1-4 to fly again", 1.0F, 0.35F, 0.25F);
    }
    if (hud.faulted) {
        text(w / 2.0 - 260, 60, 2.2, "FRAME FAULT (non-finite, capped or rejected)", 1.0F, 0.3F,
             0.3F);
    }
    if (hud.placeholders > 0) {
        (void)std::snprintf(b.data(), b.size(), "%d data gaps (placeholders)", hud.placeholders);
        text(w - 340, 250, 1.3, b.data(), 1.0F, 0.85F, 0.4F);
    }

    // Attitude indicator, bottom centre.
    const double ax = w / 2.0;
    const double ay = h - 120.0;
    glEnable(GL_SCISSOR_TEST);
    glScissor(static_cast<int>(ax - 110), 20, 220, 200);
    glPushMatrix();
    glTranslated(ax, ay, 0.0);
    glRotated(-s.roll * kDeg, 0.0, 0.0, 1.0);
    const double px_per_deg = 4.0;
    glTranslated(0.0, s.pitch * kDeg * px_per_deg, 0.0);
    glBegin(GL_QUADS);
    glColor4f(0.25F, 0.5F, 0.85F, 0.85F);
    glVertex2d(-400, -800);
    glVertex2d(400, -800);
    glVertex2d(400, 0);
    glVertex2d(-400, 0);
    glColor4f(0.55F, 0.35F, 0.15F, 0.85F);
    glVertex2d(-400, 0);
    glVertex2d(400, 0);
    glVertex2d(400, 800);
    glVertex2d(-400, 800);
    glEnd();
    glBegin(GL_LINES);
    glColor3f(1.0F, 1.0F, 1.0F);
    for (int deg = -30; deg <= 30; deg += 10) {
        const double y = -deg * px_per_deg;
        const double half = deg == 0 ? 400.0 : 30.0;
        glVertex2d(-half, y);
        glVertex2d(half, y);
    }
    glEnd();
    glPopMatrix();
    glDisable(GL_SCISSOR_TEST);
    glBegin(GL_LINES);  // the aircraft reference
    glColor3f(1.0F, 0.9F, 0.2F);
    glVertex2d(ax - 50, ay);
    glVertex2d(ax - 15, ay);
    glVertex2d(ax + 15, ay);
    glVertex2d(ax + 50, ay);
    glEnd();

    // Controls, bottom left: cyclic box (dot: applied; cross: trim point),
    // collective bar, pedal bar.
    const double cx = 90.0;
    const double cy = h - 100.0;
    panel(10, h - 190, 230, 180);
    glBegin(GL_LINE_LOOP);
    glColor3f(0.8F, 0.8F, 0.8F);
    glVertex2d(cx - 60, cy - 60);
    glVertex2d(cx + 60, cy - 60);
    glVertex2d(cx + 60, cy + 60);
    glVertex2d(cx - 60, cy + 60);
    glEnd();
    const double tx = cx + 60.0 * hud.trim_point.lat_cyclic;
    const double ty = cy - 60.0 * hud.trim_point.lon_cyclic;
    glBegin(GL_LINES);
    glColor3f(0.4F, 1.0F, 0.4F);
    glVertex2d(tx - 6, ty);
    glVertex2d(tx + 6, ty);
    glVertex2d(tx, ty - 6);
    glVertex2d(tx, ty + 6);
    glEnd();
    const double sx = cx + 60.0 * hud.applied.lat_cyclic;
    const double sy = cy - 60.0 * hud.applied.lon_cyclic;
    glPointSize(7.0F);
    glBegin(GL_POINTS);
    glColor3f(1.0F, 1.0F, 0.3F);
    glVertex2d(sx, sy);
    glEnd();
    const double colx = 175.0;
    glBegin(GL_QUADS);
    glColor3f(1.0F, 1.0F, 0.3F);
    glVertex2d(colx, cy + 60);
    glVertex2d(colx + 14, cy + 60);
    glVertex2d(colx + 14, cy + 60 - 120.0 * hud.applied.collective);
    glVertex2d(colx, cy + 60 - 120.0 * hud.applied.collective);
    glEnd();
    const double py = cy + 75.0;
    glBegin(GL_LINES);
    glColor3f(0.8F, 0.8F, 0.8F);
    glVertex2d(cx - 60, py);
    glVertex2d(cx + 60, py);
    glColor3f(0.4F, 1.0F, 0.4F);  // the trim point: where a centred twist puts the pedal
    glVertex2d(cx + 60.0 * hud.trim_point.pedal, py - 9);
    glVertex2d(cx + 60.0 * hud.trim_point.pedal, py + 9);
    glColor3f(1.0F, 1.0F, 0.3F);
    glVertex2d(cx + 60.0 * hud.applied.pedal, py - 6);
    glVertex2d(cx + 60.0 * hud.applied.pedal, py + 6);
    glEnd();
    text(colx - 6, cy + 64, 1.2, "COLL");
    if (hud.collective_held) {
        if (hud.collective_lever_known) {
            // Where the lever is, against the held collective (pickup).
            const double ly = cy + 60 - 120.0 * hud.collective_lever;
            glBegin(GL_LINES);
            glColor3f(1.0F, 0.4F, 0.3F);
            glVertex2d(colx - 6, ly);
            glVertex2d(colx + 20, ly);
            glEnd();
        }
        text(w / 2.0 - 300, 180, 2.2,
             hud.collective_lever_known ? "COLL SYNC - bring the lever to the yellow bar"
                                        : "COLL SYNC - move the collective lever",
             1.0F, 0.6F, 0.3F);
    }
    (void)std::snprintf(b.data(), b.size(), "col %.3f lat %+.3f lon %+.3f ped %+.3f",
                        hud.applied.collective, hud.applied.lat_cyclic, hud.applied.lon_cyclic,
                        hud.applied.pedal);
    text(14, h - 186, 1.1, b.data());

    if (help) {
        panel(w / 2.0 - 230, 90, 520, 290);
        text(w / 2.0 - 220, 100, 1.5,
             "Esc quit   P pause   R restart   H help\n"
             "1 pad   2 hover   3 60 kt   4 120 kt\n"
             "V view (chase / CG eye / ground observer)\n"
             "\n"
             "mouse input: hold left button to fly the cyclic\n"
             "  W/S collective   Z/X pedals   T or right button trim\n"
             "  B brakes   C trim reset   A SAS   Q pitch bias   F FPS\n"
             "gamepad (default map): right stick cyclic,\n"
             "  left stick collective (rate) and pedals, A trim,\n"
             "  B brakes, X FPS, Y trim reset, LB SAS, RB pitch bias\n"
             "\n"
             "AFCS at start: --afcs (default SAS and pitch bias).\n"
             "Trim also releases the FPS: fly there, let go.\n"
             "Banked above 60 kt with trim held: FPS turn switch (TURN).");
    }
    glDisable(GL_BLEND);
}

void GlView::draw(const HudInfo& hud) {
    int w = 1;
    int h = 1;
    glfwGetFramebufferSize(impl_->window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.55F, 0.70F, 0.88F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (hud.state != nullptr) {
        impl_->draw_world(hud, w, h);
        impl_->draw_hud(hud, w, h);
    }
    if (!impl_->screenshot.empty()) {
        std::vector<unsigned char> px(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) *
                                      3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
        if (std::FILE* f = std::fopen(impl_->screenshot.c_str(), "wb"); f != nullptr) {
            (void)std::fprintf(f, "P6\n%d %d\n255\n", w, h);
            for (int row = h - 1; row >= 0; --row) {  // GL rows run bottom up
                (void)std::fwrite(
                    px.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(w) * 3, 1,
                    static_cast<std::size_t>(w) * 3, f);
            }
            (void)std::fclose(f);
        }
        impl_->screenshot.clear();
    }
    glfwSwapBuffers(impl_->window);
}

void GlView::screenshot(std::string path) noexcept {
    impl_->screenshot = std::move(path);
}

void GlView::set_camera(int camera) noexcept {
    switch (camera) {
        case 1:
            impl_->camera = Camera::cockpit;
            break;
        case 2:
            impl_->camera = Camera::observer;
            break;
        default:
            impl_->camera = Camera::chase;
            break;
    }
}

}  // namespace station
