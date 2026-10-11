#ifndef MERCURY_TEST_SOUNDCARD_DIALOG_H
#define MERCURY_TEST_SOUNDCARD_DIALOG_H
#include <string>
struct ffaudio_interface;
int soundcard_dialog_restart_fail_closed_case(bool (*&restart_mercury_for_test)(const std::string&));
int soundcard_dialog_audio_init_fail_closed_case(const ffaudio_interface*& audio_interface_for_test);
#endif
