/**
 * @file setup_dialog.cc
 * @brief Main setup/configuration dialog implementation
 */

#include "gui/dialogs/setup_dialog.h"
#include "gui/ini_parser.h"
#include "gui/gui_state.h"
#include "common/common_defines.h"
#include "imgui.h"

#include <cstring>
#include <cstdio>
#include <cmath>

// Guard interval dropdown values (ms). Ngi = ms * 12 samples at 12kHz OFDM rate.
static const double GI_VALUES_MS[] = {
    1.333, 2.0, 2.667, 3.0, 3.333, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5
};
static const char* GI_LABELS[] = {
    "1.33 ms (Ngi=16)",
    "2.00 ms (Ngi=24)",
    "2.67 ms (Ngi=32)",
    "3.00 ms (Ngi=36) [default]",
    "3.33 ms (Ngi=40)",
    "4.00 ms (Ngi=48)",
    "4.50 ms (Ngi=54)",
    "5.00 ms (Ngi=60)",
    "5.50 ms (Ngi=66)",
    "6.00 ms (Ngi=72)",
    "6.50 ms (Ngi=78)"
};
static const int GI_COUNT = sizeof(GI_VALUES_MS) / sizeof(GI_VALUES_MS[0]);
static const int GI_DEFAULT_IDX = 3;  // 3.0ms

// Key-exchange phase -> label. Values mirror KX_* in
// include/crypto/mercury_crypto.h:43-49 (KX_IDLE=0 .. KX_ACTIVE=6). Kept local
// (literal cases) so this TU need not include the crypto header.
static const char* GetKxPhaseStringSetup(int phase) {
    switch (phase) {
        case 1: return "X25519 sent";
        case 2: return "X25519 done";
        case 3: return "ML-KEM key sent";
        case 4: return "ML-KEM ciphertext sent";
        case 5: return "deriving session key";
        case 6: return "activating";
        default: return "in progress";
    }
}

static int gi_ms_to_index(double ms) {
    int best = GI_DEFAULT_IDX;
    double best_diff = 999.0;
    for (int i = 0; i < GI_COUNT; i++) {
        double diff = fabs(GI_VALUES_MS[i] - ms);
        if (diff < best_diff) {
            best_diff = diff;
            best = i;
        }
    }
    return best;
}

// Global dialog - Meyer's Singleton to avoid static init order fiasco
SetupDialog& get_setup_dialog() {
    static SetupDialog instance;
    return instance;
}

SetupDialog::SetupDialog()
    : is_open_(false)
    , current_tab_(0)
    , radio_type_(0)
    , control_port_(7002)
    , data_port_(7003)
    , connection_timeout_ms_(15000)   // single source of truth = INI/CLI default (main.cc:788)
    , link_timeout_ms_(30000)         // single source of truth = INI/CLI default (main.cc:790)
    , max_connection_attempts_(15)    // single source of truth = INI/CLI default (main.cc:789)
    , exit_on_disconnect_(false)
    , ptt_on_delay_ms_(100)
    , ptt_off_delay_ms_(200)
    , pilot_tone_ms_(0)
    , pilot_tone_hz_(250)
    , gear_shift_enabled_(false)      // single source of truth = INI default (ini:215) & headless NO_GEAR_SHIFT (main.cc:775)
    , initial_config_(ROBUST_0)   // overwritten from g_settings on open; kept consistent with the INI default
    , ldpc_iterations_max_(50)
    , coarse_freq_sync_enabled_(false)
    , robust_mode_enabled_(false)
    , bandwidth_mode_(0)
    , guard_interval_idx_(GI_DEFAULT_IDX)
    , hide_console_(false)
    , break_fh_gate_enabled_(true)       // proven, default-ON (arq_common.cc)
    , turnaround_rephase_enabled_(true)  // proven, default-ON (arq_common.cc)
    , encryption_mode_(0)
{
    memset(my_callsign_, 0, sizeof(my_callsign_));
    strncpy(my_callsign_, "N0CALL", sizeof(my_callsign_) - 1);
    memset(psk_hex_, 0, sizeof(psk_hex_));
    memset(rate_table_path_, 0, sizeof(rate_table_path_));
}

SetupDialog::~SetupDialog() {
}

void SetupDialog::open() {
    is_open_ = true;
    loadSettings();
}

void SetupDialog::close() {
    is_open_ = false;
}

void SetupDialog::loadSettings() {
    // Load from global settings
    strncpy(my_callsign_, g_settings.my_callsign.c_str(), sizeof(my_callsign_) - 1);

    if (g_settings.radio_type == "sbitx") {
        radio_type_ = 1;
    } else {
        radio_type_ = 0;
    }

    control_port_ = g_settings.control_port;
    data_port_ = g_settings.data_port;
    connection_timeout_ms_ = g_settings.connection_timeout_ms;
    link_timeout_ms_ = g_settings.link_timeout_ms;
    max_connection_attempts_ = g_settings.max_connection_attempts;
    exit_on_disconnect_ = g_settings.exit_on_disconnect;

    ptt_on_delay_ms_ = g_settings.ptt_on_delay_ms;
    ptt_off_delay_ms_ = g_settings.ptt_off_delay_ms;
    pilot_tone_ms_ = g_settings.pilot_tone_ms;
    pilot_tone_hz_ = g_settings.pilot_tone_hz;

    gear_shift_enabled_ = g_settings.gear_shift_enabled;
    initial_config_ = g_settings.initial_config;
    ldpc_iterations_max_ = g_settings.ldpc_iterations_max;
    coarse_freq_sync_enabled_ = g_settings.coarse_freq_sync_enabled;
    robust_mode_enabled_ = g_settings.robust_mode_enabled;
    bandwidth_mode_ = g_settings.bandwidth_mode;

    guard_interval_idx_ = gi_ms_to_index(g_settings.guard_interval_ms);

    hide_console_ = g_settings.hide_console;
    log_file_enabled_ = g_settings.log_enabled;

    encryption_mode_ = g_settings.encryption_mode;
    strncpy(psk_hex_, g_settings.psk_hex.c_str(), sizeof(psk_hex_) - 1);
    psk_hex_[sizeof(psk_hex_) - 1] = 0;

    break_fh_gate_enabled_ = g_settings.break_fh_gate_enabled;
    turnaround_rephase_enabled_ = g_settings.turnaround_rephase_enabled;
    strncpy(rate_table_path_, g_settings.rate_table_path.c_str(), sizeof(rate_table_path_) - 1);
    rate_table_path_[sizeof(rate_table_path_) - 1] = 0;
}

bool SetupDialog::render() {
    if (!is_open_) return false;

    bool settings_applied = false;

    ImGui::SetNextWindowSize(ImVec2(550, 450), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Modem Setup", &is_open_, ImGuiWindowFlags_NoCollapse)) {

        // Tab bar
        if (ImGui::BeginTabBar("SetupTabs")) {

            if (ImGui::BeginTabItem("Station")) {
                renderStationTab();
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Network")) {
                renderNetworkTab();
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("ARQ")) {
                renderARQTab();
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Gear Shift")) {
                renderGearShiftTab();
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Security")) {
                renderSecurityTab();
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Advanced")) {
                renderAdvancedTab();
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Buttons
        float button_width = 100;
        float total_width = button_width * 3 + ImGui::GetStyle().ItemSpacing.x * 2;
        float start_x = (ImGui::GetWindowWidth() - total_width) / 2;

        ImGui::SetCursorPosX(start_x);
        if (ImGui::Button("OK", ImVec2(button_width, 0))) {
            applyToSettings();   // writes g_settings AND persists the INI (footgun fix)
            settings_applied = true;
            is_open_ = false;
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(button_width, 0))) {
            is_open_ = false;
        }

        ImGui::SameLine();
        if (ImGui::Button("Apply", ImVec2(button_width, 0))) {
            applyToSettings();   // writes g_settings AND persists the INI (footgun fix)
            settings_applied = true;
        }
    }
    ImGui::End();

    return settings_applied;
}

// Copy dialog fields into g_settings, then persist to disk. Previously OK/Apply
// only wrote g_settings (in-memory) and the INI was saved ONLY by the separate
// Advanced->"Save Settings to File" button or a soundcard restart, so a user who
// edited a setting and clicked OK lost it on exit. Now both OK and Apply persist.
void SetupDialog::applyToSettings() {
    g_settings.my_callsign = my_callsign_;
    g_settings.radio_type = (radio_type_ == 1) ? "sbitx" : "stockhf";
    g_settings.control_port = control_port_;
    g_settings.data_port = data_port_;
    g_settings.connection_timeout_ms = connection_timeout_ms_;
    g_settings.link_timeout_ms = link_timeout_ms_;
    g_settings.max_connection_attempts = max_connection_attempts_;
    g_settings.exit_on_disconnect = exit_on_disconnect_;
    g_settings.ptt_on_delay_ms = ptt_on_delay_ms_;
    g_settings.ptt_off_delay_ms = ptt_off_delay_ms_;
    g_settings.pilot_tone_ms = pilot_tone_ms_;
    g_settings.pilot_tone_hz = pilot_tone_hz_;
    g_settings.gear_shift_enabled = gear_shift_enabled_;
    g_settings.initial_config = initial_config_;
    g_settings.ldpc_iterations_max = ldpc_iterations_max_;
    g_gui_state.ldpc_iterations_max.store(ldpc_iterations_max_);
    g_settings.coarse_freq_sync_enabled = coarse_freq_sync_enabled_;
    g_gui_state.coarse_freq_sync_enabled.store(coarse_freq_sync_enabled_);
    g_settings.robust_mode_enabled = robust_mode_enabled_;
    g_gui_state.robust_mode_enabled.store(robust_mode_enabled_);
    g_settings.bandwidth_mode = bandwidth_mode_;
    g_gui_state.bandwidth_mode.store(bandwidth_mode_);
    // narrowband_enabled driven by bandwidth_mode: always start NB
    g_settings.narrowband_enabled = true;
    g_gui_state.narrowband_enabled.store(true);
    g_settings.guard_interval_ms = GI_VALUES_MS[guard_interval_idx_];
    g_settings.hide_console = hide_console_;
    g_settings.log_enabled = log_file_enabled_;
    g_settings.encryption_mode = encryption_mode_;
    g_gui_state.encryption_mode.store(encryption_mode_);
    g_settings.psk_hex = psk_hex_;

    // Proven features previously env-only (effective on next modem restart; the
    // env vars are applied from these INI values at startup in main.cc).
    g_settings.break_fh_gate_enabled = break_fh_gate_enabled_;
    g_settings.turnaround_rephase_enabled = turnaround_rephase_enabled_;
    g_settings.rate_table_path = rate_table_path_;

    // FOOTGUN FIX: persist immediately so OK/Apply survive exit without needing
    // the separate Advanced->"Save Settings to File" step.
    g_settings.save(getDefaultConfigPath());
}

void SetupDialog::renderStationTab() {
    ImGui::Spacing();

    ImGui::Text("My Callsign:");
    ImGui::SameLine(150);
    ImGui::SetNextItemWidth(150);
    ImGui::InputText("##callsign", my_callsign_, sizeof(my_callsign_),
                     ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank);

    ImGui::Spacing();

    ImGui::Text("Radio Type:");
    ImGui::SameLine(150);
    const char* radio_types[] = { "Stock HF (Generic)", "sBitx" };
    ImGui::SetNextItemWidth(200);
    ImGui::Combo("##radio_type", &radio_type_, radio_types, 2);

    ImGui::Spacing();
    ImGui::Spacing();

    ImGui::TextWrapped("Your callsign is used for the ARQ protocol handshake. "
                       "The radio type affects PTT and CAT control behavior.");
}

void SetupDialog::renderNetworkTab() {
    ImGui::Spacing();

    ImGui::Text("Control Port:");
    ImGui::SameLine(150);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##control_port", &control_port_);
    if (control_port_ < 1024) control_port_ = 1024;
    if (control_port_ > 65535) control_port_ = 65535;

    ImGui::Spacing();

    ImGui::Text("Data Port:");
    ImGui::SameLine(150);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##data_port", &data_port_);
    if (data_port_ < 1024) data_port_ = 1024;
    if (data_port_ > 65535) data_port_ = 65535;

    ImGui::Spacing();
    ImGui::Spacing();

    ImGui::TextWrapped("Control port is used for commands (connect, disconnect, etc.). "
                       "Data port is used for the actual data transfer. "
                       "Default ports are 7002 and 7003.");
}

void SetupDialog::renderARQTab() {
    ImGui::Spacing();

    ImGui::Text("Connection Timeout:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("ms##conn_timeout", &connection_timeout_ms_);
    if (connection_timeout_ms_ < 1000) connection_timeout_ms_ = 1000;
    if (connection_timeout_ms_ > 300000) connection_timeout_ms_ = 300000;

    ImGui::Spacing();

    ImGui::Text("Link Timeout:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("ms##link_timeout", &link_timeout_ms_);
    if (link_timeout_ms_ < 1000) link_timeout_ms_ = 1000;
    if (link_timeout_ms_ > 600000) link_timeout_ms_ = 600000;

    ImGui::Spacing();

    ImGui::Text("Max Connect Attempts:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##max_attempts", &max_connection_attempts_);
    if (max_connection_attempts_ < 1) max_connection_attempts_ = 1;
    if (max_connection_attempts_ > 100) max_connection_attempts_ = 100;

    ImGui::Spacing();

    ImGui::Checkbox("Exit on Disconnect", &exit_on_disconnect_);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // PTT Timing Section
    ImGui::Text("PTT Timing");
    ImGui::Spacing();

    ImGui::Text("PTT On Delay:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("ms##ptt_on", &ptt_on_delay_ms_);
    if (ptt_on_delay_ms_ < 0) ptt_on_delay_ms_ = 0;
    if (ptt_on_delay_ms_ > 1000) ptt_on_delay_ms_ = 1000;

    ImGui::Spacing();

    ImGui::Text("PTT Off Delay:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("ms##ptt_off", &ptt_off_delay_ms_);
    if (ptt_off_delay_ms_ < 0) ptt_off_delay_ms_ = 0;
    if (ptt_off_delay_ms_ > 1000) ptt_off_delay_ms_ = 1000;

    ImGui::Spacing();

    ImGui::Text("Pilot Tone:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(80);
    ImGui::InputInt("ms##pilot", &pilot_tone_ms_);
    if (pilot_tone_ms_ < 0) pilot_tone_ms_ = 0;
    if (pilot_tone_ms_ > 500) pilot_tone_ms_ = 500;

    ImGui::Text("Pilot Freq:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(80);
    ImGui::InputInt("Hz##pilot_freq", &pilot_tone_hz_);
    if (pilot_tone_hz_ < 100) pilot_tone_hz_ = 100;
    if (pilot_tone_hz_ > 3000) pilot_tone_hz_ = 3000;

    ImGui::Spacing();
    ImGui::Spacing();

    ImGui::TextWrapped("PTT On Delay: Time after keying PTT before audio starts. "
                       "Increase if your radio clips the start of transmissions (100-200ms typical).");

    ImGui::Spacing();

    ImGui::TextWrapped("PTT Off Delay: Time after audio ends before unkeying PTT. "
                       "Increase if your transmissions are clipped at the end (200-500ms typical).");

    ImGui::Spacing();

    ImGui::TextWrapped("Pilot Tone: Tone before OFDM to trigger RF-sensing amplifiers. "
                       "Use 250Hz (out of band) to avoid decoder interference, or 1500Hz (in band). "
                       "Set duration to 0 to disable.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextWrapped("Connection timeout is the max time to wait for a response during connection. "
                       "Link timeout is the max time without receiving data before disconnecting.");
}

void SetupDialog::renderGearShiftTab() {
    ImGui::Spacing();

    ImGui::Checkbox("Enable Gear Shifting", &gear_shift_enabled_);

    ImGui::Spacing();

    ImGui::Checkbox("Enable Robust Mode (MFSK)", &robust_mode_enabled_);
    ImGui::TextWrapped("Uses narrowband FSK for weak-signal hailing and low-speed data. "
                       "When combined with Gear Shift, hails on Robust then shifts to OFDM.");

    ImGui::Spacing();

    ImGui::Text("Bandwidth Mode:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(300);
    const char* bw_items[] = { "Auto (NB hail, WB upgrade)", "Narrowband Only (500 Hz)" };
    ImGui::Combo("##bw_mode", &bandwidth_mode_, bw_items, 2);
    ImGui::TextWrapped("Auto: all connections start narrowband. If both stations support wideband, "
                       "upgrades to 2344 Hz after connecting. NB Only: stays at 500 Hz always.");

    ImGui::Spacing();

    if (!gear_shift_enabled_) {
        ImGui::BeginDisabled();
    }

    ImGui::Text("Initial Configuration:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(300);
    const char* preview = config_to_string(initial_config_);
    if (ImGui::BeginCombo("##initial_config", preview)) {
        for (int i = 0; i < FULL_CONFIG_LADDER_SIZE; i++) {
            int cfg = FULL_CONFIG_LADDER[i];
            bool is_selected = (initial_config_ == cfg);
            if (ImGui::Selectable(config_to_string(cfg), is_selected))
                initial_config_ = cfg;
            if (is_selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    if (!gear_shift_enabled_) {
        ImGui::EndDisabled();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextWrapped("Gear shifting automatically adjusts the modulation configuration based on channel conditions. "
                       "Turboshift probes the link bidirectionally at connection time, then the success-based "
                       "ladder maintains the optimal rate during data transfer.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Performance Features");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "(proven; restart to apply)");
    ImGui::Spacing();

    ImGui::Checkbox("Turnaround re-phase (long-batch reverse-ACK)", &turnaround_rephase_enabled_);
    ImGui::TextWrapped("Re-centers the reverse-ACK listen window later on long OFDM batches so the "
                       "end-of-batch SACK lands in window. Proven throughput fix. Leave ON.");

    ImGui::Spacing();

    ImGui::Checkbox("BREAK forward-health gate", &break_fh_gate_enabled_);
    ImGui::TextWrapped("Suppresses spurious BREAK->ROBUST_0 collapses while the forward link is healthy. "
                       "Proven fix. Leave ON.");

    ImGui::Spacing();

    ImGui::Text("Rate table:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(300);
    ImGui::InputText("##rate_table_path", rate_table_path_, sizeof(rate_table_path_));
    ImGui::TextWrapped("Optional path to an effective-rate optimizer table (JSON). "
                       "Leave empty to use the modem's built-in default search.");
}

void SetupDialog::renderAdvancedTab() {
    ImGui::Spacing();

    ImGui::Text("OFDM Guard Interval");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Guard Interval:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(300);
    if (ImGui::BeginCombo("##gi", GI_LABELS[guard_interval_idx_])) {
        for (int i = 0; i < GI_COUNT; i++) {
            bool is_selected = (guard_interval_idx_ == i);
            if (ImGui::Selectable(GI_LABELS[i], is_selected))
                guard_interval_idx_ = i;
            if (is_selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "(restart)");

    ImGui::Spacing();

    ImGui::TextWrapped("Shorter GI = faster throughput but less multipath tolerance. "
                       "4.5ms covers most HF channels (ITU moderate+poor). "
                       "Reduce for loopback/VHF. Both stations must match. Requires restart.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("LDPC Decoder");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Max Iterations:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(150);
    ImGui::SliderInt("##ldpc_iter", &ldpc_iterations_max_, 5, 50, "%d");

    ImGui::Spacing();

    ImGui::TextWrapped("Lower values reduce CPU load on slower hardware but may miss marginal frames. "
                       "Recommended: 50 for desktop, 15-25 for low-power devices. "
                       "Requires modem restart.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Frequency Sync");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Checkbox("Enable Coarse Frequency Search", &coarse_freq_sync_enabled_);

    ImGui::Spacing();

    ImGui::TextWrapped("Searches +/-30 Hz for crystal oscillator drift between HF radios. "
                       "Disable for loopback or same-clock setups to save CPU.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (ImGui::Checkbox("Hide console window", &hide_console_)) {
        // Value changed
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "(requires restart)");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Logging");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Checkbox("Enable log file output", &log_file_enabled_);

    ImGui::Spacing();
    ImGui::TextWrapped("Captures all console output to a timestamped log file. "
                       "Logs are saved to %%APPDATA%%\\Mercury\\logs\\ (Windows) "
                       "or ~/.config/mercury/logs/ (Linux). Requires restart to take effect.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Save/Load buttons
    if (ImGui::Button("Save Settings to File")) {
        std::string path = getDefaultConfigPath();
        if (g_settings.save(path)) {
            // Success - could show a notification
        }
    }

    ImGui::SameLine();

    if (ImGui::Button("Load Settings from File")) {
        std::string path = getDefaultConfigPath();
        if (g_settings.load(path)) {
            loadSettings();  // Refresh dialog with loaded values
        }
    }
}

void SetupDialog::renderSecurityTab() {
    ImGui::Spacing();

    ImGui::Text("Encryption");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Encryption Mode:");
    ImGui::SameLine(180);
    ImGui::SetNextItemWidth(300);
    const char* enc_modes[] = {
        "Off (plaintext)",
        "Strict (hold data until PQ key exchange)",
        "Fast (classical-first, PQ upgrade later)"
    };
    ImGui::Combo("##enc_mode", &encryption_mode_, enc_modes, 3);

    ImGui::Spacing();

    if (encryption_mode_ == 0)
        ImGui::BeginDisabled();

    ImGui::TextWrapped(
        "Strict: X25519 + ML-KEM-768 hybrid key exchange completes before any data is sent. "
        "Provides post-quantum security from the first byte. "
        "Fast: X25519 key exchange enables ChaCha20-Poly1305 immediately, then "
        "upgrades to hybrid PQ in the background.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Pre-Shared Key (PSK):");
    ImGui::Spacing();
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##psk_hex", psk_hex_, sizeof(psk_hex_),
                     ImGuiInputTextFlags_CharsHexadecimal);

    ImGui::Spacing();
    ImGui::TextWrapped(
        "Optional authentication key (hex string, up to 64 bytes / 128 hex chars). "
        "Both stations must use the same PSK to connect. "
        "When set, prevents unauthorized stations from establishing encrypted sessions. "
        "Leave empty for unauthenticated encryption.");

    if (encryption_mode_ == 0)
        ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Show current encryption status
    int active_mode = g_gui_state.encryption_mode.load();
    bool enc_active = g_gui_state.encryption_active.load();
    if (active_mode > 0 && enc_active) {
        bool pq_active = g_gui_state.encryption_pq_active.load();
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f),
                           pq_active ? "Encryption ACTIVE (PQ hybrid: X25519 + ML-KEM-768)"
                                     : "Encryption ACTIVE (classical: X25519)");
        // Session fingerprint for out-of-band voice verification — both stations
        // should read the same value aloud to confirm there is no MITM.
        if (g_gui_state.enc_fingerprint_valid.load()) {
            char fp[24];
            {
                GuiLockGuard lk(g_gui_state.enc_fingerprint_mutex);
                snprintf(fp, sizeof(fp), "%s", g_gui_state.enc_fingerprint);
            }
            ImGui::Spacing();
            ImGui::Text("Session fingerprint (verify out-of-band):");
            ImGui::TextColored(ImVec4(0.55f, 0.85f, 0.65f, 1.0f), "  %s", fp);
        }
    } else if (active_mode > 0) {
        // During the handshake, show the live KX phase so STRICT is not opaque.
        int kxp = g_gui_state.kx_phase.load();
        if (kxp > 0 && kxp < 6)
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f),
                               "Key exchange: %s", GetKxPhaseStringSetup(kxp));
        else
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f),
                               "Encryption enabled (waiting for connection)");
    } else {
        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "Encryption disabled");
    }

    ImGui::Spacing();
    ImGui::TextWrapped(
        "Note: Encryption settings take effect on the next connection. "
        "Both stations must have encryption enabled to establish an encrypted session. "
        "Requires modem restart to change mode.");
}
