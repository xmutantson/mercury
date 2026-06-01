import math

# Engine params (mercury, confirmed from code):
# - passband sample rate: 48000 Hz
# - frequency_interpolation_rate = 4  -> baseband rate = 48000/4 = 12000 Hz
# - MFSK symbol = exactly ONE Nfft FFT window (baud-scaling-spike.md §1)
# - Nofdm = Nfft + Ngi; Ngi = Nfft*gi (gi ratio constant under baud-scaling)
# - The COHERENT INTEGRATION window per MFSK symbol = Nfft baseband samples
#   (the FFT spans Nfft samples; GI is discarded at RX so does NOT contribute
#    to coherent integration). This is the quantity vs coherence time.
# - Full symbol PERIOD (incl GI) = Nofdm baseband samples (airtime, not integ window)

fs_pass = 48000.0
interp  = 4
fs_base = fs_pass / interp   # 12000 Hz baseband
gi_ratio = 54.0/256.0        # physical_config.cc:37 default (Ngi=54 @ Nfft256)
# NOTE: also compute for the 3.0ms / gi=36 variant if ROBUST configs override; the
# coherent-integration window (Nfft) is gi-INDEPENDENT, so only the period changes.

print("="*78)
print("SYMBOL DURATION vs COHERENCE TIME  (MFSK = one Nfft FFT window)")
print("="*78)
print(f"baseband rate fs = 48000/{interp} = {fs_base:.0f} Hz")
print()

# Coherence time. Standard relation: T_coh ~= 1/(2*pi*sigma_doppler) (rms) or the
# common engineering approx T_coh ~ 1/Doppler_spread (for the order-of-magnitude
# wall). We report BOTH a strict (1/(2*pi*sigma)) and the loose (1/f_d) bound, and
# the FST4 rule (Doppler spread < tone spacing; best when < 1/8 tone spacing).
#
# Per WSJT-X FST4 doc: "frequency spread defined as TWICE the std dev of the Gaussian
# Doppler spectrum". So a profile labeled "0.5 Hz Doppler" => sigma_f = 0.25 Hz if
# "0.5" is the FULL spread (2-sigma), OR sigma_f = 0.5 if "0.5" is the spread param.
# Task labels: moderate 0.5 Hz, poor 1 Hz. We treat these as the Doppler SPREAD f_d
# (the conventional CCIR/ITU spread param) and report T_coh = 1/f_d (engineering) and
# the Clarke/Jakes-like 0.423/f_d as a tighter coherence-time (0.5-correlation).

dopplers = {"moderate (0.5 Hz)": 0.5, "poor (1 Hz)": 1.0, "poor-edge (2 Hz)": 2.0}

print("Coherence-time estimates per profile:")
print(f"{'profile':<20}{'f_d (Hz)':>10}{'1/f_d (s)':>12}{'0.423/f_d (s)':>15}{'1/(2pi*f_d) (s)':>17}")
for name, fd in dopplers.items():
    print(f"{name:<20}{fd:>10.2f}{1.0/fd:>12.3f}{0.423/fd:>15.3f}{1.0/(2*math.pi*fd):>17.4f}")
print()

print("Per-baud symbol geometry + comparison to coherence time:")
hdr = f"{'K':>2}{'Nfft':>6}{'Ngi':>5}{'Nofdm':>7}{'integ win T_fft (ms)':>22}{'symbol period (ms)':>20}{'tone sp (Hz)':>13}"
print(hdr)
for K in [1,2,4,8]:
    Nfft = 256*K
    Ngi  = round(Nfft*gi_ratio)
    Nofdm = Nfft + Ngi
    T_fft_ms = 1000.0 * Nfft / fs_base          # coherent integration window (GI excluded)
    T_sym_ms = 1000.0 * Nofdm / fs_base          # full symbol period (airtime)
    tone_sp  = fs_base / Nfft                      # subcarrier/tone spacing
    print(f"{K:>2}{Nfft:>6}{Ngi:>5}{Nofdm:>7}{T_fft_ms:>22.2f}{T_sym_ms:>20.2f}{tone_sp:>13.3f}")
print()

print("FST4-rule check (Doppler spread vs tone spacing):")
print("  Rule (WSJT-X FST4 quick-start): decode requires f_d < tone_spacing;")
print("  sensitivity best when f_d <= tone_spacing/8.")
print(f"{'K':>2}{'tone sp (Hz)':>13}{'tone/8 (Hz)':>13}{'  f_d=0.5 ok?':>16}{'  f_d=1 ok?':>14}{'  f_d=2 ok?':>14}")
for K in [1,2,4,8]:
    Nfft = 256*K
    tone_sp = fs_base / Nfft
    t8 = tone_sp/8.0
    def status(fd):
        if fd < t8: return "BEST"
        elif fd < tone_sp: return "ok(degr)"
        else: return "FAIL"
    print(f"{K:>2}{tone_sp:>13.3f}{t8:>13.3f}{status(0.5):>16}{status(1.0):>14}{status(2.0):>14}")
print()

print("Coherence-time-vs-integration-window ratio (T_coh / T_fft):")
print("  ratio >> 1 : channel ~static over the FFT -> coherent gain HOLDS")
print("  ratio ~ 1  : channel decorrelates within FFT -> gain ERODES")
print("  ratio < 1  : intra-symbol fading -> irreducible error floor (coherence WALL)")
print(f"{'K':>2}{'T_fft(ms)':>11}{'  mod 1/f_d=2000ms':>20}{'  poor 1/f_d=1000ms':>21}{'  2Hz 1/f_d=500ms':>19}")
for K in [1,2,4,8]:
    Nfft = 256*K
    T_fft = 1000.0*Nfft/fs_base
    print(f"{K:>2}{T_fft:>11.2f}{2000.0/T_fft:>20.1f}{1000.0/T_fft:>21.1f}{500.0/T_fft:>19.1f}")
