/**
 * @file soundcard_dialog.cc
 * @brief Sound card selection dialog implementation
 */

#include "gui/dialogs/soundcard_dialog.h"
#include "gui/ini_parser.h"
#include "gui/gui_state.h"
#include "imgui.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <unistd.h>
#include <climits>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ffaudio for device enumeration
extern "C" {
#include "ffaudio/audio.h"
#ifdef _WIN32
extern const ffaudio_interface ffwasapi;
extern const ffaudio_interface ffdsound;
#elif defined(__APPLE__)
extern const ffaudio_interface ffcoreaudio;
#else  // Linux
extern const ffaudio_interface ffalsa;
extern const ffaudio_interface ffpulse;
#endif
}

static const ffaudio_interface* audio_interface_for_test = nullptr;
static int failing_audio_init_calls = 0;
static bool (*restart_mercury_for_test)(const std::string&) = nullptr;
static int failing_restart_calls = 0;

static int failing_audio_init(ffaudio_init_conf*) {
    ++failing_audio_init_calls;
    return -1;
}

static bool failing_restart(const std::string&) {
    ++failing_restart_calls;
    return false;
}

// Global dialog - Meyer's Singleton to avoid static init order fiasco
SoundCardDialog& get_soundcard_dialog() {
    static SoundCardDialog instance;
    return instance;
}

// Helper to restart Mercury
#ifdef _WIN32
bool restartMercuryProcess(const char* executable_path, std::FILE* diagnostic) {
    STARTUPINFOA si = {};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi = {};
    if (!CreateProcessA(executable_path, NULL, NULL, NULL, FALSE, 0, NULL, NULL,
                        &si, &pi)) {
        const DWORD error = GetLastError();
        fprintf(diagnostic,
                "ERROR: Failed to restart Mercury: CreateProcessA failed "
                "with Windows error %lu.\n",
                static_cast<unsigned long>(error));
        return false;
    }

    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    g_gui_state.request_shutdown.store(true);
    return true;
}
#endif

bool restartMercury(const std::string& config_path) {
    // Save settings before restart
    if (!g_settings.save(config_path)) {
        fprintf(stderr, "ERROR: Failed to save settings to %s; restart aborted.\n",
                config_path.c_str());
        return false;
    }

#ifdef _WIN32
    // Get the path to the current executable
    char exePath[MAX_PATH];
    const DWORD exe_path_length = GetModuleFileNameA(NULL, exePath, MAX_PATH);
    if (exe_path_length == 0 || exe_path_length >= MAX_PATH) {
        const DWORD error = GetLastError();
        fprintf(stderr,
                "ERROR: Failed to restart Mercury: GetModuleFileNameA failed "
                "with Windows error %lu.\n",
                static_cast<unsigned long>(error));
        return false;
    }

    // Launch new instance
    return restartMercuryProcess(exePath, stderr);
#elif defined(__APPLE__)
    // macOS: use _NSGetExecutablePath
    char exePath[PATH_MAX];
    uint32_t size = sizeof(exePath);
    if (_NSGetExecutablePath(exePath, &size) == 0) {
        pid_t pid = fork();
        if (pid == 0) {
            char* argv[] = { exePath, NULL };
            execv(exePath, argv);
            _exit(1);
        } else if (pid > 0) {
            g_gui_state.request_shutdown.store(true);
            return true;
        }
    }
#else
    // Linux: get executable path via /proc/self/exe
    char exePath[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    if (len > 0) {
        exePath[len] = '\0';
        pid_t pid = fork();
        if (pid == 0) {
            char* argv[] = { exePath, NULL };
            execv(exePath, argv);
            _exit(1);
        } else if (pid > 0) {
            g_gui_state.request_shutdown.store(true);
            return true;
        }
    }
#endif

    return false;
}

SoundCardDialog::SoundCardDialog()
    : is_open_(false)
    , devices_enumerated_(false)
    , selected_input_device_(0)
    , selected_output_device_(0)
    , selected_input_channel_(0)
    , selected_output_channel_(0)
    , selected_audio_system_(0)
    , temp_input_device_(0)
    , temp_output_device_(0)
    , temp_input_channel_(0)
    , temp_output_channel_(0)
    , temp_audio_system_(0)
{
}

SoundCardDialog::~SoundCardDialog() {
}

void SoundCardDialog::open() {
    is_open_ = true;

    // Load settings from g_settings
#ifdef _WIN32
    temp_audio_system_ = (g_settings.audio_system == "wasapi") ? 0 : 1;
#elif defined(__APPLE__)
    temp_audio_system_ = 0;  // CoreAudio only
#else  // Linux
    temp_audio_system_ = (g_settings.audio_system == "alsa") ? 0 : 1;
#endif
    temp_input_channel_ = g_settings.input_channel;
    temp_output_channel_ = g_settings.output_channel;

    // Always refresh devices when opening to ensure we have the current list
    devices_enumerated_ = false;
    refreshDevices();

    // Find the matching input device by name
    temp_input_device_ = 0;  // Default to first device
    for (size_t i = 0; i < input_devices_.size(); i++) {
        if (input_devices_[i].name == g_settings.input_device) {
            temp_input_device_ = (int)i;
            break;
        }
    }

    // Find the matching output device by name
    temp_output_device_ = 0;  // Default to first device
    for (size_t i = 0; i < output_devices_.size(); i++) {
        if (output_devices_[i].name == g_settings.output_device) {
            temp_output_device_ = (int)i;
            break;
        }
    }

    // Sync the "confirmed" selections with temp
    selected_input_device_ = temp_input_device_;
    selected_output_device_ = temp_output_device_;
    selected_input_channel_ = temp_input_channel_;
    selected_output_channel_ = temp_output_channel_;
    selected_audio_system_ = temp_audio_system_;
}

void SoundCardDialog::close() {
    is_open_ = false;
}

void SoundCardDialog::refreshDevices() {
    devices_enumerated_ = false;
    input_devices_.clear();
    output_devices_.clear();

    // Select audio interface based on current system
#ifdef _WIN32
    const ffaudio_interface* audio = (temp_audio_system_ == 0) ? &ffwasapi : &ffdsound;
#elif defined(__APPLE__)
    const ffaudio_interface* audio = &ffcoreaudio;
#else  // Linux
    const ffaudio_interface* audio = (temp_audio_system_ == 0) ? &ffalsa : &ffpulse;
#endif
    if (audio_interface_for_test) {
        audio = audio_interface_for_test;
    }

    // Initialize audio subsystem
    ffaudio_init_conf init_conf = {};
    init_conf.app_name = "Mercury";
    if (audio->init(&init_conf) != 0) {
        return;
    }

    // Enumerate capture (input) devices
    ffaudio_dev* dev = audio->dev_alloc(FFAUDIO_DEV_CAPTURE);
    if (dev) {
        while (audio->dev_next(dev) == 0) {
            AudioDeviceInfo info;
            const char* name = audio->dev_info(dev, FFAUDIO_DEV_NAME);
            const char* id = audio->dev_info(dev, FFAUDIO_DEV_ID);
            const char* is_default = audio->dev_info(dev, FFAUDIO_DEV_IS_DEFAULT);

            info.name = name ? name : "Unknown Device";
            info.id = id ? id : "";
            info.is_default = (is_default != nullptr);
            info.channels = 0;  // Unknown by default
            info.sample_rate = 0;

#ifdef _WIN32
            // Try to get device format info (WASAPI only)
            if (temp_audio_system_ == 0) {
                const char* mix_fmt = audio->dev_info(dev, FFAUDIO_DEV_MIX_FORMAT);
                if (mix_fmt) {
                    const unsigned int* fmt = (const unsigned int*)mix_fmt;
                    info.sample_rate = fmt[1];
                    info.channels = fmt[2];
                }
            }
#endif

            input_devices_.push_back(info);
        }
        audio->dev_free(dev);
    }

    // Enumerate playback (output) devices
    dev = audio->dev_alloc(FFAUDIO_DEV_PLAYBACK);
    if (dev) {
        while (audio->dev_next(dev) == 0) {
            AudioDeviceInfo info;
            const char* name = audio->dev_info(dev, FFAUDIO_DEV_NAME);
            const char* id = audio->dev_info(dev, FFAUDIO_DEV_ID);
            const char* is_default = audio->dev_info(dev, FFAUDIO_DEV_IS_DEFAULT);

            info.name = name ? name : "Unknown Device";
            info.id = id ? id : "";
            info.is_default = (is_default != nullptr);
            info.channels = 0;  // Unknown by default
            info.sample_rate = 0;

#ifdef _WIN32
            // Try to get device format info (WASAPI only)
            if (temp_audio_system_ == 0) {
                const char* mix_fmt = audio->dev_info(dev, FFAUDIO_DEV_MIX_FORMAT);
                if (mix_fmt) {
                    const unsigned int* fmt = (const unsigned int*)mix_fmt;
                    info.sample_rate = fmt[1];
                    info.channels = fmt[2];
                }
            }
#endif

            output_devices_.push_back(info);
        }
        audio->dev_free(dev);
    }

    audio->uninit();

    // Ensure at least one entry exists
    if (input_devices_.empty()) {
        input_devices_.push_back({"Default Input Device", "", true, 0, 0});
    }
    if (output_devices_.empty()) {
        output_devices_.push_back({"Default Output Device", "", true, 0, 0});
    }

    devices_enumerated_ = true;
}

bool SoundCardDialog::acceptAndRestart(const std::string& config_path) {
#ifdef __APPLE__
    const int audio_system_count = 1;
#else
    const int audio_system_count = 2;
#endif
    if (!devices_enumerated_
            || temp_input_device_ < 0
            || temp_input_device_ >= (int)input_devices_.size()
            || temp_output_device_ < 0
            || temp_output_device_ >= (int)output_devices_.size()
            || temp_input_channel_ < 0 || temp_input_channel_ > 2
            || temp_output_channel_ < 0 || temp_output_channel_ > 2
            || temp_audio_system_ < 0 || temp_audio_system_ >= audio_system_count) {
        return false;
    }

    const MercurySettings previous_settings = g_settings;

    int in_ch_validated = temp_input_channel_;
    g_settings.input_device = input_devices_[temp_input_device_].name;
    if (input_devices_[temp_input_device_].channels == 1) {
        in_ch_validated = 0;
    }

    int out_ch_validated = temp_output_channel_;
    g_settings.output_device = output_devices_[temp_output_device_].name;
    if (output_devices_[temp_output_device_].channels == 1) {
        out_ch_validated = 2;
    }

    g_settings.input_channel = in_ch_validated;
    g_settings.output_channel = out_ch_validated;
#ifdef _WIN32
    g_settings.audio_system = (temp_audio_system_ == 0) ? "wasapi" : "dsound";
#elif defined(__APPLE__)
    g_settings.audio_system = "coreaudio";
#else  // Linux
    g_settings.audio_system = (temp_audio_system_ == 0) ? "alsa" : "pulse";
#endif

    const bool restarted = restart_mercury_for_test
                         ? restart_mercury_for_test(config_path)
                         : restartMercury(config_path);
    if (!restarted) {
        g_settings = previous_settings;
        if (!config_path.empty() && !g_settings.save(config_path)) {
            fprintf(stderr,
                    "ERROR: Failed to restore settings after restart failure: %s.\n",
                    config_path.c_str());
        }
        return false;
    }

    selected_input_device_ = temp_input_device_;
    selected_output_device_ = temp_output_device_;
    selected_audio_system_ = temp_audio_system_;
    selected_input_channel_ = in_ch_validated;
    selected_output_channel_ = out_ch_validated;
    is_open_ = false;
    return true;
}

int soundcard_dialog_restart_fail_closed_selftest() {
    const MercurySettings saved_settings = g_settings;
    const std::string original_input = "Original Input Device";
    const std::string original_output = "Original Output Device";
    const std::string candidate_input = "Candidate Input Device";
    const std::string candidate_output = "Candidate Output Device";
    g_settings.input_device = original_input;
    g_settings.output_device = original_output;
    g_settings.input_channel = 1;
    g_settings.output_channel = 0;

    SoundCardDialog dialog;
    dialog.is_open_ = true;
    dialog.devices_enumerated_ = true;
    dialog.input_devices_.push_back({candidate_input, "input-id", false, 1, 48000});
    dialog.output_devices_.push_back({candidate_output, "output-id", false, 1, 48000});
    dialog.temp_input_device_ = 0;
    dialog.temp_output_device_ = 0;
    dialog.temp_input_channel_ = 1;
    dialog.temp_output_channel_ = 0;

    failing_restart_calls = 0;
    restart_mercury_for_test = failing_restart;
    const bool accepted = dialog.acceptAndRestart(std::string());
    restart_mercury_for_test = nullptr;

    const bool passed = failing_restart_calls == 1
                     && !accepted
                     && dialog.isOpen()
                     && g_settings.input_device == original_input
                     && g_settings.output_device == original_output
                     && g_settings.input_channel == 1
                     && g_settings.output_channel == 0;

    g_settings = saved_settings;
    return passed ? 0 : 1;
}

int soundcard_dialog_audio_init_fail_closed_selftest() {
#ifdef _WIN32
    const ffaudio_interface* system_audio = &ffwasapi;
#elif defined(__APPLE__)
    const ffaudio_interface* system_audio = &ffcoreaudio;
#else
    const ffaudio_interface* system_audio = &ffalsa;
#endif
    ffaudio_interface failing_audio = *system_audio;
    failing_audio.init = failing_audio_init;

    const MercurySettings saved_settings = g_settings;
    const std::string original_input = "Original Input Device";
    const std::string original_output = "Original Output Device";
    g_settings.input_device = original_input;
    g_settings.output_device = original_output;

    failing_audio_init_calls = 0;
    audio_interface_for_test = &failing_audio;
    SoundCardDialog dialog;
    dialog.open();
    audio_interface_for_test = nullptr;

    // This is the same action invoked by the "OK & Restart" button.  The empty
    // path would also make restart harmless if the fail-closed guard regressed.
    const bool accepted = dialog.acceptAndRestart(std::string());
    const bool passed = failing_audio_init_calls == 1
                     && !dialog.devices_enumerated_
                     && dialog.input_devices_.empty()
                     && dialog.output_devices_.empty()
                     && !accepted
                     && dialog.isOpen()
                     && g_settings.input_device == original_input
                     && g_settings.output_device == original_output;

    audio_interface_for_test = nullptr;
    g_settings = saved_settings;
    return passed ? 0 : 1;
}

bool SoundCardDialog::render() {
    if (!is_open_) return false;

    bool settings_applied = false;

    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Sound Card Settings", &is_open_, ImGuiWindowFlags_NoCollapse)) {

        // Audio System Selection
        ImGui::Text("Audio System:");
        ImGui::SameLine(150);
#ifdef _WIN32
        const char* audio_systems[] = { "WASAPI (Recommended)", "DirectSound" };
        int n_audio_systems = 2;
#elif defined(__APPLE__)
        const char* audio_systems[] = { "CoreAudio" };
        int n_audio_systems = 1;
#else  // Linux
        const char* audio_systems[] = { "ALSA", "PulseAudio" };
        int n_audio_systems = 2;
#endif
        int prev_system = temp_audio_system_;
        if (ImGui::Combo("##audio_system", &temp_audio_system_, audio_systems, n_audio_systems)) {
            if (prev_system != temp_audio_system_) {
                devices_enumerated_ = false;
                refreshDevices();
                temp_input_device_ = 0;
                temp_output_device_ = 0;
            }
        }

        ImGui::Separator();
        ImGui::Spacing();

        // Input Device
        ImGui::Text("Input Device:");
        ImGui::SameLine(150);

        // Build combo items string with device capabilities
        std::string input_combo;
        for (size_t i = 0; i < input_devices_.size(); i++) {
            input_combo += input_devices_[i].name;
            // Show channel info if available
            if (input_devices_[i].channels > 0) {
                input_combo += " [";
                input_combo += (input_devices_[i].channels == 1) ? "Mono" : "Stereo";
                if (input_devices_[i].sample_rate > 0) {
                    input_combo += ", " + std::to_string(input_devices_[i].sample_rate / 1000) + "kHz";
                }
                input_combo += "]";
            }
            if (input_devices_[i].is_default) {
                input_combo += " (Default)";
            }
            input_combo += '\0';
        }
        input_combo += '\0';

        ImGui::SetNextItemWidth(350);
        if (ImGui::Combo("##input_device", &temp_input_device_, input_combo.c_str())) {
            // When device changes, validate channel selection
            if (temp_input_device_ >= 0 && temp_input_device_ < (int)input_devices_.size()) {
                int ch = input_devices_[temp_input_device_].channels;
                if (ch == 1 && temp_input_channel_ != 0) {
                    temp_input_channel_ = 0;  // Force LEFT for mono
                }
            }
        }

        // Input Channel - show options based on device capabilities
        ImGui::Text("Input Channel:");
        ImGui::SameLine(150);

        // Get selected input device channel count
        int input_ch_count = 0;
        if (temp_input_device_ >= 0 && temp_input_device_ < (int)input_devices_.size()) {
            input_ch_count = input_devices_[temp_input_device_].channels;
        }

        ImGui::SetNextItemWidth(150);
        if (input_ch_count == 1) {
            // Mono device - force mono, show as disabled
            const char* mono_options[] = { "Mono (device is mono)" };
            int mono_sel = 0;
            ImGui::Combo("##input_channel", &mono_sel, mono_options, 1);
            temp_input_channel_ = 0;  // Force LEFT (mono)
        } else if (input_ch_count >= 2) {
            // Stereo device - show all options
            const char* channels[] = { "Left", "Right", "Stereo (L+R)" };
            ImGui::Combo("##input_channel", &temp_input_channel_, channels, 3);
        } else {
            // Unknown - show all options with warning
            const char* channels[] = { "Left", "Right", "Stereo (L+R)" };
            ImGui::Combo("##input_channel", &temp_input_channel_, channels, 3);
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.0f, 1.0f), "(?)");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Device channel count unknown.\nIf unsure, try 'Left' for most radio interfaces.");
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Output Device
        ImGui::Text("Output Device:");
        ImGui::SameLine(150);

        std::string output_combo;
        for (size_t i = 0; i < output_devices_.size(); i++) {
            output_combo += output_devices_[i].name;
            // Show channel info if available
            if (output_devices_[i].channels > 0) {
                output_combo += " [";
                output_combo += (output_devices_[i].channels == 1) ? "Mono" : "Stereo";
                if (output_devices_[i].sample_rate > 0) {
                    output_combo += ", " + std::to_string(output_devices_[i].sample_rate / 1000) + "kHz";
                }
                output_combo += "]";
            }
            if (output_devices_[i].is_default) {
                output_combo += " (Default)";
            }
            output_combo += '\0';
        }
        output_combo += '\0';

        ImGui::SetNextItemWidth(350);
        if (ImGui::Combo("##output_device", &temp_output_device_, output_combo.c_str())) {
            // When device changes, validate channel selection
            if (temp_output_device_ >= 0 && temp_output_device_ < (int)output_devices_.size()) {
                int ch = output_devices_[temp_output_device_].channels;
                if (ch == 1) {
                    temp_output_channel_ = 2;  // Force STEREO mode for mono device
                }
            }
        }

        // Output Channel - show options based on device capabilities
        ImGui::Text("Output Channel:");
        ImGui::SameLine(150);

        // Get selected output device channel count
        int output_ch_count = 0;
        if (temp_output_device_ >= 0 && temp_output_device_ < (int)output_devices_.size()) {
            output_ch_count = output_devices_[temp_output_device_].channels;
        }

        ImGui::SetNextItemWidth(150);
        if (output_ch_count == 1) {
            // Mono device - force mono, show as disabled
            const char* mono_options[] = { "Mono (device is mono)" };
            int mono_sel = 0;
            ImGui::Combo("##output_channel", &mono_sel, mono_options, 1);
            temp_output_channel_ = 2;  // Force STEREO mode (will output to mono)
        } else if (output_ch_count >= 2) {
            // Stereo device - show all options
            const char* out_channels[] = { "Left", "Right", "Stereo (Both)" };
            ImGui::Combo("##output_channel", &temp_output_channel_, out_channels, 3);
        } else {
            // Unknown - show all options with warning
            const char* out_channels[] = { "Left", "Right", "Stereo (Both)" };
            ImGui::Combo("##output_channel", &temp_output_channel_, out_channels, 3);
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.0f, 1.0f), "(?)");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Device channel count unknown.\n'Stereo (Both)' is recommended for most setups.");
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Refresh button
        if (ImGui::Button("Refresh Devices")) {
            devices_enumerated_ = false;
            refreshDevices();
        }

        // Info text
        ImGui::Spacing();
#ifdef _WIN32
        ImGui::TextWrapped("WASAPI is recommended for virtual audio cables and provides better latency. "
                           "DirectSound may be more compatible with older hardware.");
#elif defined(__APPLE__)
        ImGui::TextWrapped("CoreAudio is the native macOS audio system.");
#else
        ImGui::TextWrapped("PulseAudio is recommended for desktop Linux. "
                           "ALSA provides direct hardware access with lower latency.");
#endif
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
            "Tip: Most radio USB interfaces are mono. If you see [Mono], channel selection is automatic.");
        if (!devices_enumerated_) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                "Audio device initialization failed. Settings were not changed.");
        }

        ImGui::Spacing();
        ImGui::Spacing();

        // Buttons
        float button_width = 100;
        float total_width = button_width * 3 + ImGui::GetStyle().ItemSpacing.x * 2;
        float start_x = (ImGui::GetWindowWidth() - total_width) / 2;

        ImGui::SetCursorPosX(start_x);
        if (ImGui::Button("OK & Restart", ImVec2(button_width + 20, 0))) {
            settings_applied = acceptAndRestart(getDefaultConfigPath());
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(button_width, 0))) {
            is_open_ = false;
        }
    }
    ImGui::End();

    return settings_applied;
}
