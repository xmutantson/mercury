/**
 * @file gui_main.h
 * @brief Mercury HF Modem GUI - Public interface
 */

#ifndef GUI_MAIN_H_
#define GUI_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the GUI (create window, OpenGL context, ImGui)
 * @return 0 on success, -1 on failure
 */
int gui_init(void);

/**
 * @brief Run the GUI main loop (blocking)
 * @return 0 on normal exit
 */
int gui_main_loop(void);

/**
 * @brief Shutdown the GUI and cleanup resources
 */
void gui_shutdown(void);

/**
 * @brief GUI thread entry point (for use with pthread_create)
 * @param arg Optional gui_thread_context used to publish initialization status
 * @return nullptr
 */
void* gui_thread_func(void* arg);

#ifdef __cplusplus
}

#include <atomic>
#include <cstdio>

enum gui_startup_status {
    GUI_STARTUP_PENDING = 0,
    GUI_STARTUP_SUCCEEDED,
    GUI_STARTUP_FAILED
};

struct gui_thread_context {
    explicit gui_thread_context(int (*initializer)(void) = gui_init)
        : status(GUI_STARTUP_PENDING), init(initializer) {}

    std::atomic<int> status;
    int (*init)(void);
};

/**
 * @brief Validate the GUI thread's published startup result.
 * @return 0 on success, -1 after emitting an error on failure
 */
int gui_validate_startup(int status, FILE* error_stream);
#endif

#endif // GUI_MAIN_H_
