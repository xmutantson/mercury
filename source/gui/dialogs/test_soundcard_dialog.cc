// Directed sound card regression cases.
#include "gui/dialogs/soundcard_dialog.h"
#include "gui/ini_parser.h"
#include "test_soundcard_dialog.h"

extern "C" {
#include "ffaudio/audio.h"
#ifdef _WIN32
extern const ffaudio_interface ffwasapi;
#elif defined(__APPLE__)
extern const ffaudio_interface ffcoreaudio;
#else
extern const ffaudio_interface ffalsa;
#endif
}

static int failing_audio_init_calls = 0;
int failing_restart_calls = 0;
static int failing_audio_init(ffaudio_init_conf*) {
    ++failing_audio_init_calls;
    return -1;
}

bool failing_restart(const std::string&) {
    ++failing_restart_calls;
    return false;
}

int soundcard_dialog_audio_init_fail_closed_case(const ffaudio_interface*& audio_interface_for_test) {
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
