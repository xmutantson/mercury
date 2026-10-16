/**
 * @file waterfall.cc
 * @brief Waterfall spectrum display implementation
 */

#include "gui/widgets/waterfall.h"
#include "gui/gui_state.h"
#include "imgui.h"

#include <GL/gl.h>
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif
#include <cmath>
#include <cstring>
#include <cstdio>

// Global waterfall - Meyer's Singleton to avoid static init order fiasco
WaterfallDisplay& get_waterfall() {
    static WaterfallDisplay instance;
    return instance;
}

// Mutex for thread-safe sample pushing - also lazy init
static GuiMutex& get_waterfall_mutex() {
    static GuiMutex instance;
    return instance;
}
#define waterfall_mutex (get_waterfall_mutex())

WaterfallDisplay::WaterfallDisplay()
    : sample_index_(0)
    , history_index_(0)
    , min_db_(-60.0f)
    , max_db_(0.0f)
    , texture_id_(0)
    , texture_data_(nullptr)
    , initialized_(false)
    , enabled_(true)
{
    memset(sample_buffer_, 0, sizeof(sample_buffer_));
    memset(fft_magnitudes_, 0, sizeof(fft_magnitudes_));
    for (int i = 0; i < WATERFALL_HISTORY_LINES; i++) {
        for (int j = 0; j < WATERFALL_DISPLAY_BINS; j++) {
            history_[i][j] = min_db_;
        }
    }
}

WaterfallDisplay::~WaterfallDisplay() {
    shutdown();
}

bool WaterfallDisplay::init() {
    if (initialized_) return true;

    // Allocate texture data (RGB, width=DISPLAY_BINS, height=HISTORY_LINES)
    int width = WATERFALL_DISPLAY_BINS;
    int height = WATERFALL_HISTORY_LINES;
    texture_data_ = new unsigned char[width * height * 3];
    memset(texture_data_, 0, width * height * 3);

    // Create OpenGL texture
    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data_);

    initialized_ = true;
    return true;
}

void WaterfallDisplay::shutdown() {
    if (texture_id_) {
        glDeleteTextures(1, &texture_id_);
        texture_id_ = 0;
    }
    if (texture_data_) {
        delete[] texture_data_;
        texture_data_ = nullptr;
    }
    initialized_ = false;
}

// Capture thread: hand the samples over and return. It never waits for the GUI
// (display samples are dropped if the GUI thread holds the lock or has not
// drained the queue) and never runs an FFT; render does that on the GUI thread.
void WaterfallDisplay::pushSamples(const double* samples, int count) {
    // Skip all processing when waterfall is disabled (major CPU savings)
    if (!enabled_.load(std::memory_order_relaxed) || count <= 0) return;

    if (!waterfall_mutex.try_lock()) {
        return;
    }
    const int capacity = WATERFALL_FFT_SIZE * 2;
    const int room = capacity - pending_count_;
    const int n = count < room ? count : room;
    if (n > 0) {
        memcpy(pending_ + pending_count_, samples, (size_t)n * sizeof(double));
        pending_count_ += n;
    }
    waterfall_mutex.unlock();
}

// GUI thread: take the queued samples under the lock (a copy only), then run
// the FFTs and update the history outside it. sample_buffer_ and history_ are
// touched only by the GUI thread.
void WaterfallDisplay::drainPending() {
    int n;
    {
        GuiLockGuard lock(waterfall_mutex);
        n = pending_count_;
        if (n > 0)
            memcpy(drain_scratch_, pending_, (size_t)n * sizeof(double));
        pending_count_ = 0;
    }
    for (int i = 0; i < n; i++) {
        sample_buffer_[sample_index_] = drain_scratch_[i];
        sample_index_++;

        // When buffer is full, process FFT
        if (sample_index_ >= WATERFALL_FFT_SIZE) {
            processFFT();
            sample_index_ = 0;
        }
    }
}

void WaterfallDisplay::processFFT() {
    // Copy samples to complex array
    std::complex<double> fft_data[WATERFALL_FFT_SIZE];
    for (int i = 0; i < WATERFALL_FFT_SIZE; i++) {
        // Apply Hanning window
        double window = 0.5 * (1.0 - cos(2.0 * 3.14159265358979 * i / (WATERFALL_FFT_SIZE - 1)));
        fft_data[i] = std::complex<double>(sample_buffer_[i] * window, 0.0);
    }

    // Perform FFT
    fft(fft_data, WATERFALL_FFT_SIZE);

    // Calculate magnitudes in dB (only positive frequencies)
    for (int i = 0; i < WATERFALL_FFT_SIZE / 2; i++) {
        double mag = std::abs(fft_data[i]) / (WATERFALL_FFT_SIZE / 2);
        if (mag < 1e-10) mag = 1e-10;
        fft_magnitudes_[i] = (float)(20.0 * log10(mag));
    }

    // Store in history (only the display bins: 0 to DISPLAY_MAX_HZ)
    for (int i = 0; i < WATERFALL_DISPLAY_BINS; i++) {
        history_[history_index_][i] = fft_magnitudes_[i];
    }
    history_index_ = (history_index_ + 1) % WATERFALL_HISTORY_LINES;
}

// Cooley-Tukey FFT (in-place, radix-2, DIT, iterative)
void WaterfallDisplay::fft(std::complex<double>* v, int n) {
    if (n <= 1) return;

    // Bit-reversal permutation
    int j = 0;
    for (int i = 0; i < n; ++i) {
        if (i < j) {
            std::complex<double> tmp = v[i];
            v[i] = v[j];
            v[j] = tmp;
        }
        int m = n >> 1;
        while (m >= 1 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Iterative butterfly passes
    for (int len = 2; len <= n; len <<= 1) {
        double angle = -2.0 * 3.14159265358979 / (double)len;
        std::complex<double> wlen(cos(angle), sin(angle));
        for (int i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            int half = len >> 1;
            for (int k = 0; k < half; ++k) {
                std::complex<double> u = v[i + k];
                std::complex<double> t = w * v[i + k + half];
                v[i + k] = u + t;
                v[i + k + half] = u - t;
                w *= wlen;
            }
        }
    }
}

// Jet colormap: blue -> cyan -> green -> yellow -> red
static void jetColormap(float value, unsigned char* rgb) {
    // value should be 0.0 to 1.0
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;

    float r, g, b;
    if (value < 0.25f) {
        r = 0.0f;
        g = 4.0f * value;
        b = 1.0f;
    } else if (value < 0.5f) {
        r = 0.0f;
        g = 1.0f;
        b = 1.0f - 4.0f * (value - 0.25f);
    } else if (value < 0.75f) {
        r = 4.0f * (value - 0.5f);
        g = 1.0f;
        b = 0.0f;
    } else {
        r = 1.0f;
        g = 1.0f - 4.0f * (value - 0.75f);
        b = 0.0f;
    }

    rgb[0] = (unsigned char)(r * 255);
    rgb[1] = (unsigned char)(g * 255);
    rgb[2] = (unsigned char)(b * 255);
}

void WaterfallDisplay::render(float width, float height) {
    if (!initialized_) {
        init();
    }

    int tex_width = WATERFALL_DISPLAY_BINS;
    int tex_height = WATERFALL_HISTORY_LINES;

    drainPending();

    // History and FFT state belong to the GUI thread.
    {

        for (int y = 0; y < tex_height; y++) {
            // Map y to history index (scroll effect)
            int hist_idx = (history_index_ - 1 - y + WATERFALL_HISTORY_LINES) % WATERFALL_HISTORY_LINES;

            for (int x = 0; x < tex_width; x++) {
                float db = history_[hist_idx][x];
                float normalized = (db - min_db_) / (max_db_ - min_db_);

                unsigned char* pixel = &texture_data_[(y * tex_width + x) * 3];
                jetColormap(normalized, pixel);
            }
        }
    }

    // Upload to GPU
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tex_width, tex_height, GL_RGB, GL_UNSIGNED_BYTE, texture_data_);

    // Draw using ImGui
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Draw the texture
    draw_list->AddImage(
        (ImTextureID)(intptr_t)texture_id_,
        pos,
        ImVec2(pos.x + width, pos.y + height),
        ImVec2(0, 0),
        ImVec2(1, 1)
    );

    // Draw frequency scale (0 to WATERFALL_DISPLAY_MAX_HZ)
    char label_mid[32], label_max[32];
    snprintf(label_mid, sizeof(label_mid), "%d Hz", WATERFALL_DISPLAY_MAX_HZ / 2);
    snprintf(label_max, sizeof(label_max), "%d Hz", WATERFALL_DISPLAY_MAX_HZ);

    draw_list->AddText(ImVec2(pos.x, pos.y + height + 2), IM_COL32(200, 200, 200, 255), "0 Hz");
    draw_list->AddText(ImVec2(pos.x + width / 2 - 20, pos.y + height + 2), IM_COL32(200, 200, 200, 255), label_mid);
    draw_list->AddText(ImVec2(pos.x + width - 45, pos.y + height + 2), IM_COL32(200, 200, 200, 255), label_max);

    // Reserve space
    ImGui::Dummy(ImVec2(width, height + 20));
}

void WaterfallDisplay::setRange(float min_db, float max_db) {
    min_db_ = min_db;
    max_db_ = max_db;
}

void WaterfallDisplay::setEnabled(bool enabled) {
    enabled_.store(enabled, std::memory_order_relaxed);
}

// Thread-safe wrapper for pushing samples (extern "C" for C linkage)
extern "C" void waterfall_push_samples(const double* samples, int count) {
    g_waterfall.pushSamples(samples, count);
}

#ifndef _WIN32
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

int waterfall_nonblocking_case() {
#ifdef _WIN32
    printf("[TEST-WATERFALL-NONBLOCKING] SKIP: POSIX process backstop\n");
    return 0;
#else
    int failures = 0;
    for (int mode = 0; mode < 2; ++mode) {
        fflush(stdout);
        const pid_t pid = fork();
        if (pid == 0) {
            alarm(5); // Deadlock backstop, never a latency acceptance threshold.
            WaterfallDisplay display;
            double samples[WATERFALL_FFT_SIZE * 3];
            for (int i = 0; i < WATERFALL_FFT_SIZE * 3; ++i)
                samples[i] = 0.1 * sin(0.05 * i);
            if (mode == 0) {
                waterfall_mutex.lock();
                display.pushSamples(samples, WATERFALL_FFT_SIZE);
                waterfall_mutex.unlock();
                // A contended display update must leave capture free to proceed.
                _exit(display.history_index_ == 0 ? 0 : 11);
            }
            display.pushSamples(samples, WATERFALL_FFT_SIZE * 3);
            if (display.history_index_ != 0 || display.sample_index_ != 0)
                _exit(12); // The producer performed FFT work.
            if (display.pending_count_ != WATERFALL_FFT_SIZE * 2
                || memcmp(display.pending_, samples, sizeof(display.pending_)) != 0)
                _exit(13); // Handoff exceeds capacity or changes accepted samples.
            display.drainPending();
            if (display.history_index_ != 2 || display.pending_count_ != 0)
                _exit(14);
            // Recover from contention/full queue, preserving a partial FFT frame.
            display.pushSamples(samples, 17);
            display.drainPending();
            display.setEnabled(false);
            display.pushSamples(samples, WATERFALL_FFT_SIZE);
            display.drainPending();
            if (display.history_index_ != 2 || display.sample_index_ != 17)
                _exit(15);
            display.setEnabled(true);
            display.pushSamples(samples + 17, WATERFALL_FFT_SIZE - 17);
            display.drainPending();
            if (display.history_index_ != 3 || display.sample_index_ != 0)
                _exit(16);
            _exit(0);
        }
        int status = 0;
        const bool waited = pid > 0 && waitpid(pid, &status, 0) == pid;
        const bool passed = waited && WIFEXITED(status) && WEXITSTATUS(status) == 0;
        printf("[TEST-WATERFALL-NONBLOCKING] mode=%d signal=%d exit=%d: %s\n",
            mode, waited && WIFSIGNALED(status) ? WTERMSIG(status) : 0,
            waited && WIFEXITED(status) ? WEXITSTATUS(status) : -1,
            passed ? "PASS" : "FAIL");
        if (!passed) ++failures;
    }
    return failures;
#endif
}
