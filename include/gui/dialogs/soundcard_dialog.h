/**
 * @file soundcard_dialog.h
 * @brief Sound card selection dialog
 */

#ifndef SOUNDCARD_DIALOG_H_
#define SOUNDCARD_DIALOG_H_

#include <cstdio>
#include <string>
#include <vector>

/**
 * @struct AudioDeviceInfo
 * @brief Information about an audio device
 */
struct AudioDeviceInfo {
    std::string name;
    std::string id;
    bool is_default;
    int channels;       // Number of channels (1=mono, 2=stereo, 0=unknown)
    int sample_rate;    // Sample rate (0=unknown)
};

/**
 * @class SoundCardDialog
 * @brief Dialog for selecting audio input/output devices
 */
class SoundCardDialog {
public:
    SoundCardDialog();
    ~SoundCardDialog();

    /**
     * @brief Open the dialog
     */
    void open();

    /**
     * @brief Close the dialog
     */
    void close();

    /**
     * @brief Check if dialog is open
     */
    bool isOpen() const { return is_open_; }

    /**
     * @brief Render the dialog (call each frame)
     * @return true if settings were applied
     */
    bool render();

    /**
     * @brief Refresh device lists
     */
    void refreshDevices();

    // Current selections (indices into device lists)
    int selected_input_device_;
    int selected_output_device_;
    int selected_input_channel_;   // 0=LEFT, 1=RIGHT, 2=STEREO
    int selected_output_channel_;
    int selected_audio_system_;    // 0=WASAPI, 1=DirectSound

private:
    bool acceptAndRestart(const std::string& config_path);

    friend int soundcard_dialog_restart_fail_closed_selftest();
    friend int soundcard_dialog_audio_init_fail_closed_selftest();

    bool is_open_;
    bool devices_enumerated_;

    std::vector<AudioDeviceInfo> input_devices_;
    std::vector<AudioDeviceInfo> output_devices_;

    // Temporary selections (before Apply)
    int temp_input_device_;
    int temp_output_device_;
    int temp_input_channel_;
    int temp_output_channel_;
    int temp_audio_system_;
};

// Global dialog - Meyer's Singleton to avoid static init order fiasco
SoundCardDialog& get_soundcard_dialog();
#define g_soundcard_dialog (get_soundcard_dialog())

// Save settings and restart Mercury. Returns false without restarting when the
// settings cannot be saved.
bool restartMercury(const std::string& config_path);

// Directed regression used by the modem's --test battery.
int soundcard_dialog_restart_fail_closed_selftest();
int soundcard_dialog_audio_init_fail_closed_selftest();
#ifdef _WIN32
// Launch the replacement process and request shutdown only after Windows has
// accepted it. Exposed separately so the failure path can be tested without
// saving settings or starting a second modem.
bool restartMercuryProcess(const char* executable_path, std::FILE* diagnostic);
#endif

#endif // SOUNDCARD_DIALOG_H_
