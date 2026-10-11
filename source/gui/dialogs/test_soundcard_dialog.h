#ifndef MERCURY_TEST_SOUNDCARD_DIALOG_H
#define MERCURY_TEST_SOUNDCARD_DIALOG_H
#include <string>
struct ffaudio_interface;
int soundcard_dialog_audio_init_fail_closed_case(const ffaudio_interface*& audio_interface_for_test);
extern int failing_restart_calls;
bool failing_restart(const std::string&);
#endif
