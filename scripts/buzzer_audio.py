#!/usr/bin/env python3
"""Convert a song or a sound into a clip the tests play through the buzzer of the Micras v1 board.

The buzzer is a FST-5030 magnetic transducer switched by a transistor from TIM15 on PA2. The tests
turn its PWM into an 8-bit DAC: a carrier at three times the sample rate whose duty cycle follows
the samples, which the coil and its flyback diode average into a current. This script prepares the song for that path:

1. Decode any file ffmpeg reads, cut the excerpt and mix it to mono.
2. Move the bass the buzzer cannot reproduce into harmonics it can, so that the ear still hears
   the beat through the missing fundamental.
3. Remove what is left below the high-pass cutoff, which would only waste the duty cycle range.
4. Equalize against the frequency response of the buzzer from its datasheet, taming the
   resonances at 1.5 and 4.5 kHz so that the rest of the spectrum is heard.
5. Compress and limit the result so that it stays near full scale, since loudness is the scarcest
   resource of a 5 mm buzzer.
6. Quantize to 8 bits with the quantization noise pushed away from 3.5 kHz, where the buzzer and
   the ear are most sensitive, and ramp the duty cycle from zero into the middle of its range and
   back so that playback starts and ends without a thump.

The output is tests/include/clips/<name>.hpp, which a test plays with ClipPlayer, and optionally WAV
previews of the excerpt, of the signal sent to the buzzer and of an approximation of what the buzzer
does to it.

Example:
    .venv/bin/python scripts/buzzer_audio.py chatuba.mp3 --name chatuba --start 0:32 --duration 40 --preview preview
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import subprocess
import sys
import wave
from fractions import Fraction
from pathlib import Path

import numpy as np
from scipy import signal
from scipy.ndimage import maximum_filter1d, uniform_filter1d

WORKING_RATE = 48000
"""Rate at which the song is decoded and the bass is processed, in Hz."""

FLASH_BUDGET = 1_000_000
"""Bytes of flash left for the samples: the 1 MiB of the STM32H725RG less about 48 KiB for code."""

MAX_SAMPLE_RATE = 22050
"""Highest rate worth storing: the buzzer response ends around 10 kHz."""

MIN_SAMPLE_RATE = 11000
"""Lowest rate the script accepts, which keeps the carrier at three times it above 33 kHz."""

RAMP_SECONDS = 0.25
"""Duration of the duty cycle ramps before and after the song."""

FADE_SECONDS = 0.02
"""Duration of the fade in and out of the audio itself, inside the ramps."""

BUZZER_RESPONSE = [
    (200, 67.5), (300, 69.4), (400, 70.6), (500, 71.6), (600, 74.8), (700, 70.6), (800, 70.6),
    (880, 77.0), (1000, 71.0), (1200, 70.5), (1460, 82.0), (1700, 76.0), (2000, 72.4), (2500, 69.0),
    (3000, 71.2), (3500, 74.8), (3800, 81.0), (4000, 85.7), (4500, 84.5), (5000, 84.0), (5500, 81.0),
    (6000, 78.4), (6500, 75.8), (7000, 75.0), (7500, 76.6), (8000, 77.8), (9000, 77.0), (10000, 79.0),
]
"""Sound pressure of the FST-5030 at 10 cm in dB, read from the curve on page 3 of its datasheet.

The curve was swept with a square wave, whose harmonics reach the resonances, so below about 1.4 kHz
it overstates the response to a sine; estimate_sine_response removes the harmonics.
"""


def parse_time(text: str) -> float:
    """Parse seconds given as `SS`, `MM:SS` or `HH:MM:SS`, with optional decimals."""
    seconds = 0.0

    for part in text.split(":"):
        seconds = seconds * 60 + float(part)

    return seconds


def decode(path: Path, start: float, duration: float | None) -> np.ndarray:
    """Decode an excerpt of an audio file to mono float samples at the working rate."""
    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg was not found on the PATH")

    command = ["ffmpeg", "-v", "error", "-ss", str(start)]

    if duration is not None:
        command += ["-t", str(duration)]

    command += ["-i", str(path), "-ac", "1", "-ar", str(WORKING_RATE), "-f", "f32le", "-"]
    result = subprocess.run(command, capture_output=True, check=False)

    if result.returncode != 0:
        sys.exit(f"ffmpeg failed: {result.stderr.decode(errors='replace').strip()}")

    samples = np.frombuffer(result.stdout, dtype=np.float32).astype(np.float64)

    if samples.size == 0:
        sys.exit("The excerpt is empty, check --start and --duration")

    return samples


def envelope(x: np.ndarray, rate: float, seconds: float) -> np.ndarray:
    """Smooth the magnitude of a signal with a one pole low-pass filter."""
    pole = math.exp(-1.0 / (seconds * rate))
    return signal.lfilter([1 - pole], [1, -pole], np.abs(x))


def enhance_bass(x: np.ndarray, rate: float, crossover: float, band: tuple[float, float], gain: float) -> np.ndarray:
    """Replace the bass below the crossover with odd harmonics of it in a band the buzzer reproduces.

    The bass is saturated at its own envelope, which turns each tone into something close to a
    square wave of the same loudness contour, and the harmonics that fall in the band are kept. They
    are scaled to the power of the bass they stand for.
    """
    if gain <= 0:
        return np.zeros_like(x)

    bass = signal.sosfilt(signal.butter(4, crossover, "lowpass", fs=rate, output="sos"), x)
    level = envelope(bass, rate, 0.02) + 1e-6
    saturated = level * np.tanh(4.0 * bass / level)
    bandpass = signal.butter(4, band, "bandpass", fs=rate, output="sos")
    harmonics = signal.sosfilt(bandpass, saturated)

    bass_power = np.sqrt(np.mean(bass**2))
    harmonic_power = np.sqrt(np.mean(harmonics**2)) + 1e-12

    return harmonics * gain * bass_power / harmonic_power


def square_response_db(frequencies: np.ndarray) -> np.ndarray:
    """Interpolate the datasheet curve on a logarithmic frequency axis, falling at 40 dB per decade
    above its last point."""
    known = np.array(BUZZER_RESPONSE)
    clipped = np.clip(frequencies, known[0, 0], known[-1, 0])
    inside = np.interp(np.log(clipped), np.log(known[:, 0]), known[:, 1])
    above = np.maximum(frequencies / known[-1, 0], 1.0)
    return inside - 40 * np.log10(above)


def estimate_sine_response() -> tuple[np.ndarray, np.ndarray]:
    """Estimate the response of the buzzer to a sine from the square wave curve of the datasheet.

    A square wave at f carries odd harmonics at n f with 1/n of the amplitude of the fundamental,
    and the datasheet reading at f is the power sum of the response to all of them. Subtracting the
    harmonics leaves the response to the fundamental; where they explain the whole reading, the
    estimate is floored 25 dB below it. The subtraction is repeated until it settles, and the result
    is smoothed over a third of an octave, since the curve was read by eye.
    """
    frequencies = np.geomspace(100, 12000, 480)
    measured = square_response_db(frequencies)
    estimate = measured.copy()
    orders = np.arange(3, 41, 2)

    for _ in range(30):
        harmonics = np.zeros_like(frequencies)

        for order in orders:
            at_harmonic = np.interp(np.log(order * frequencies), np.log(frequencies), estimate)
            beyond = order * frequencies > frequencies[-1]
            at_harmonic[beyond] = square_response_db(order * frequencies[beyond])
            harmonics += 10 ** (at_harmonic / 10) / order**2

        remaining = 10 ** (measured / 10) - harmonics
        floor = measured - 25
        estimate = np.maximum(10 * np.log10(np.maximum(remaining, 1e-12)), floor)

    per_octave = len(frequencies) / np.log2(frequencies[-1] / frequencies[0])
    width = max(int(round(per_octave / 3)), 1)
    padded = np.pad(estimate, width, mode="edge")
    smoothed = np.convolve(padded, np.ones(width) / width, mode="same")[width:-width]

    return frequencies, smoothed


SINE_FREQUENCIES, SINE_RESPONSE = estimate_sine_response()


def response_db(frequencies: np.ndarray) -> np.ndarray:
    """Estimated response of the buzzer to a sine in dB, on a logarithmic frequency axis."""
    clipped = np.clip(frequencies, SINE_FREQUENCIES[0], SINE_FREQUENCIES[-1])
    return np.interp(np.log(clipped), np.log(SINE_FREQUENCIES), SINE_RESPONSE)


def design_fir(rate: float, gains_db, taps: int = 255) -> np.ndarray:
    """Design a linear phase FIR filter from a gain in dB given as a function of frequency."""
    frequencies = np.linspace(0, rate / 2, 512)
    gains = 10 ** (gains_db(frequencies) / 20)
    return signal.firwin2(taps, frequencies, gains, fs=rate)


def equalizer(rate: float, strength: float, max_cut: float, max_boost: float) -> np.ndarray:
    """Design the filter that flattens the buzzer response by a fraction of its deviation.

    The reference is the median of the response between 1 and 10 kHz, so the peaks are cut and
    the dips raised around it, within the given limits.
    """
    band = np.geomspace(1000, 10000, 200)
    reference = np.median(response_db(band))

    def gains(frequencies):
        return np.clip(-strength * (response_db(frequencies) - reference), -max_cut, max_boost)

    return design_fir(rate, gains)


def buzzer_model(rate: float) -> np.ndarray:
    """Design a filter with the relative response of the buzzer, for the audible preview."""
    peak = np.max(SINE_RESPONSE)

    def gains(frequencies):
        below = np.clip(frequencies / 100.0, 1e-3, 1.0)
        return response_db(frequencies) - peak + 40 * np.log10(below)

    return design_fir(rate, gains, taps=511)


def smooth_reduction(target: np.ndarray, attack_pole: float, release_pole: float) -> np.ndarray:
    """Follow a gain reduction in dB with one time constant when it rises and another when it falls."""
    reduction = np.empty_like(target)
    current = 0.0

    for index, value in enumerate(target.tolist()):
        pole = attack_pole if value > current else release_pole
        current = value + pole * (current - value)
        reduction[index] = current

    return reduction


def compress(x: np.ndarray, rate: float, threshold_db: float, ratio: float, attack: float, release: float) -> np.ndarray:
    """Apply a feed forward compressor driven by the RMS of the signal over 10 ms."""
    level_db = 10 * np.log10(envelope(x**2, rate, 0.01) + 1e-18)
    target = np.maximum(level_db - threshold_db, 0) * (1 - 1 / ratio)
    attack_pole = math.exp(-1.0 / (attack * rate))
    release_pole = math.exp(-1.0 / (release * rate))

    return x * 10 ** (-smooth_reduction(target, attack_pole, release_pole) / 20)


def limit(x: np.ndarray, rate: float, ceiling_db: float, lookahead: float, release: float) -> np.ndarray:
    """Apply a look ahead limiter that keeps every peak under the ceiling.

    The reduction each sample needs is spread over the look ahead window before it, as a linear
    ramp, and decays with the release time constant after it, never falling below what is needed.
    """
    size = max(int(round(lookahead * rate)), 1)
    needed = np.maximum(20 * np.log10(np.abs(x) + 1e-12) - ceiling_db, 0)
    ahead = maximum_filter1d(needed, size=size, origin=-(size // 2))
    ramped = uniform_filter1d(ahead, size=size, origin=(size - 1) // 2)
    ramped = np.maximum(ramped, needed)
    release_pole = math.exp(-1.0 / (release * rate))
    reduction = np.empty_like(ramped)
    current = 0.0

    for index, value in enumerate(ramped.tolist()):
        current = max(value, current * release_pole)
        reduction[index] = current

    return x * 10 ** (-reduction / 20)


def soft_clip(x: np.ndarray, drive_db: float) -> np.ndarray:
    """Saturate a signal peaking at 1 so that its quiet parts rise by up to the drive."""
    drive = 10 ** (drive_db / 20)
    return np.tanh(drive * x) / math.tanh(drive)


def resample(x: np.ndarray, source: float, target: float) -> np.ndarray:
    """Resample with a polyphase filter, which also removes what lies above the new Nyquist rate."""
    fraction = Fraction(int(round(target)), int(round(source))).limit_denominator(2000)
    return signal.resample_poly(x, fraction.numerator, fraction.denominator)


def quantize(x: np.ndarray, rate: float, notch: float) -> np.ndarray:
    """Quantize a signal in [-1, 1] to codes of 0 to 255 around 127.5.

    The error feeds back through a damped notch, so the noise it leaves is low near the notch
    frequency and high near DC and the Nyquist rate. The error is bounded so that clipping at the
    ends of the range cannot drive the feedback unstable.
    """
    radius = 0.9
    first = 2 * radius * math.cos(2 * math.pi * notch / rate)
    second = -radius * radius
    target = 127.5 + 127.5 * x
    codes = np.empty(x.size, dtype=np.uint8)
    previous = 0.0
    before = 0.0

    for index, value in enumerate(target.tolist()):
        wanted = value + first * previous + second * before
        code = min(max(int(math.floor(wanted + 0.5)), 0), 255)
        error = min(max(wanted - code, -1.0), 1.0)
        before, previous = previous, error
        codes[index] = code

    return codes


def ramp(count: int, rising: bool) -> np.ndarray:
    """Smooth step between 0 and 1 over a number of samples."""
    t = np.linspace(0, 1, count)
    step = t * t * (3 - 2 * t)
    return step if rising else step[::-1]


def write_wav(path: Path, x: np.ndarray, rate: int) -> None:
    """Write a mono 16-bit WAV file from samples in [-1, 1]."""
    data = np.clip(x, -1, 1)
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(rate)
        output.writeframes((data * 32767).astype("<i2").tobytes())


def write_header(path: Path, name: str, codes: np.ndarray, rate: int, source: str) -> None:
    """Write the samples as a header with the clip in namespace micras::clips::<name>."""
    lines = []
    values = codes.tolist()

    for start in range(0, len(values), 32):
        lines.append("    " + ",".join(str(value) for value in values[start : start + 32]) + ",")

    body = "\n".join(lines)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"""/**
 * @file
 *
 * Generated by scripts/buzzer_audio.py from {source}.
 */

#ifndef MICRAS_CLIPS_{name.upper()}_HPP
#define MICRAS_CLIPS_{name.upper()}_HPP

#include <array>
#include <cstdint>

namespace micras::clips::{name} {{
/**
 * @brief Rate of the samples in Hz.
 */
inline constexpr uint32_t sample_rate{{{rate}}};

/**
 * @brief Samples, as duty cycles of the buzzer from 0 to 255.
 */
// clang-format off
inline constexpr std::array<uint8_t, {codes.size}> samples{{
{body}
}};
// clang-format on
}}  // namespace micras::clips::{name}

#endif  // MICRAS_CLIPS_{name.upper()}_HPP
""",
        encoding="ascii",
    )


def shape(song: np.ndarray, rate: int, settings: argparse.Namespace) -> np.ndarray:
    """Turn the decoded excerpt into a signal in [-1, 1] at the sample rate, ready to quantize."""
    song = song - np.mean(song)
    bass = enhance_bass(song, WORKING_RATE, settings.crossover, tuple(settings.bass_band), settings.bass)
    highpass = signal.butter(4, settings.highpass, "highpass", fs=WORKING_RATE, output="sos")
    shaped = signal.sosfilt(highpass, song) + bass

    x = resample(shaped, WORKING_RATE, rate)

    if settings.eq > 0:
        taps = equalizer(rate, settings.eq, settings.max_cut, settings.max_boost)
        x = signal.fftconvolve(x, taps, mode="same")

    x = x / (np.sqrt(np.mean(x**2)) + 1e-12) * 10 ** (-18 / 20)

    if settings.compression > 1:
        x = compress(x, rate, -24.0, settings.compression, attack=0.005, release=0.15)
        x = x / (np.sqrt(np.mean(x**2)) + 1e-12) * 10 ** (-12 / 20)

    x = limit(x, rate, -0.5, lookahead=0.002, release=0.06)
    x = x / (np.max(np.abs(x)) + 1e-12)

    if settings.drive > 0:
        x = soft_clip(x, settings.drive)

    return x


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="audio file in any format ffmpeg decodes")
    parser.add_argument("--name", required=True, help="name of the clip, a C++ identifier such as chatuba")
    parser.add_argument("--start", type=parse_time, default=0.0, help="start of the excerpt, SS or MM:SS")
    parser.add_argument("--duration", type=float, default=None, help="length of the excerpt in seconds")
    parser.add_argument("--rate", type=int, default=None, help="sample rate in Hz, the highest that fits by default")
    parser.add_argument("--output", type=Path, default=None, help="header to write, tests/include/clips/<name>.hpp by default")
    parser.add_argument("--preview", type=Path, default=None, help="directory for the WAV previews")
    parser.add_argument("--bass", type=float, default=1.0, help="level of the synthesized bass harmonics, 0 disables")
    parser.add_argument("--crossover", type=float, default=150.0, help="upper end of the bass that is replaced, in Hz")
    parser.add_argument(
        "--bass-band", type=float, nargs=2, default=(600.0, 2400.0), help="band of the bass harmonics in Hz"
    )
    parser.add_argument("--highpass", type=float, default=600.0, help="cutoff of the high-pass filter in Hz")
    parser.add_argument("--eq", type=float, default=0.85, help="fraction of the buzzer response that is flattened")
    parser.add_argument("--max-cut", type=float, default=12.0, help="largest cut of the equalizer in dB")
    parser.add_argument("--max-boost", type=float, default=6.0, help="largest boost of the equalizer in dB")
    parser.add_argument("--compression", type=float, default=4.0, help="ratio of the compressor, 1 disables it")
    parser.add_argument("--drive", type=float, default=2.0, help="saturation after the limiter in dB, 0 disables it")
    parser.add_argument("--budget", type=int, default=FLASH_BUDGET, help="bytes of flash available for the samples")
    arguments = parser.parse_args()

    if not re.fullmatch(r"[a-z_][a-z0-9_]*", arguments.name):
        sys.exit("--name must be a lower case C++ identifier, such as chatuba")

    if arguments.output is None:
        arguments.output = root / "tests" / "include" / "clips" / f"{arguments.name}.hpp"

    song = decode(arguments.input, arguments.start, arguments.duration)
    seconds = song.size / WORKING_RATE
    total_seconds = seconds + 2 * RAMP_SECONDS

    if arguments.rate is None:
        rate = min(MAX_SAMPLE_RATE, int(arguments.budget / total_seconds) // 50 * 50)
    else:
        rate = arguments.rate

    if rate < MIN_SAMPLE_RATE:
        sys.exit(
            f"{seconds:.1f} s would need {rate} Hz to fit in {arguments.budget} bytes; "
            f"shorten the excerpt to {arguments.budget / MIN_SAMPLE_RATE - 2 * RAMP_SECONDS:.0f} s or less"
        )

    if rate * total_seconds > arguments.budget:
        sys.exit(f"{seconds:.1f} s at {rate} Hz takes {rate * total_seconds:.0f} bytes, over {arguments.budget}")

    x = shape(song, rate, arguments)

    fade = int(FADE_SECONDS * rate)
    x[:fade] *= ramp(fade, rising=True)
    x[-fade:] *= ramp(fade, rising=False)

    ramp_count = int(RAMP_SECONDS * rate)
    audio_codes = quantize(x, rate, notch=3500.0)
    rising = np.round(127.0 * ramp(ramp_count, rising=True)).astype(np.uint8)
    falling = np.round(127.0 * ramp(ramp_count, rising=False)).astype(np.uint8)
    codes = np.concatenate([rising, audio_codes, falling])

    write_header(
        arguments.output,
        arguments.name,
        codes,
        rate,
        f"{arguments.input.name}, {arguments.start:.2f} s to {arguments.start + seconds:.2f} s",
    )

    clipped = np.mean((audio_codes == 0) | (audio_codes == 255)) * 100
    rms = np.sqrt(np.mean(x**2))
    print(f"Excerpt:     {seconds:.2f} s from {arguments.start:.2f} s")
    print(f"Sample rate: {rate} Hz, carrier {3 * rate / 1000:.1f} kHz")
    print(f"Flash:       {codes.size} bytes of {arguments.budget}")
    print(f"Level:       RMS {20 * np.log10(rms + 1e-12):.1f} dBFS, {clipped:.2f} % of samples at the rails")
    print(f"Header:      {arguments.output}")

    if arguments.preview is not None:
        arguments.preview.mkdir(parents=True, exist_ok=True)
        sent = (codes.astype(np.float64) - 127.5) / 127.5
        heard = signal.fftconvolve(sent - np.mean(sent), buzzer_model(rate), mode="same")
        heard = heard / (np.max(np.abs(heard)) + 1e-12)
        original = resample(song, WORKING_RATE, rate)
        original = original / (np.max(np.abs(original)) + 1e-12)
        write_wav(arguments.preview / "original.wav", original, rate)
        write_wav(arguments.preview / "buzzer_input.wav", sent, rate)
        write_wav(arguments.preview / "buzzer_simulated.wav", heard, rate)
        print(f"Previews:    {arguments.preview}/original.wav, buzzer_input.wav, buzzer_simulated.wav")


if __name__ == "__main__":
    main()
