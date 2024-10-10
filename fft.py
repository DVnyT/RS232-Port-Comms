import numpy as np
import matplotlib as plt
def calculate_and_plot_fft(data_points, scope_x_increment):
    """Calculate the FFT of the data and plot it."""
    # Calculate FFT
    N = len(data_points)
    fft_result = np.fft.fft(data_points)
    fft_freq = np.fft.fftfreq(N, d=scope_x_increment)  # Frequency axis
    fft_magnitude = np.abs(fft_result)  # Magnitude of the FFT

    # Plot the frequency-domain representation
    plt.figure(figsize=(14, 7))  # Width, Height in inches
    plt.plot(fft_freq[:N // 2], fft_magnitude[:N // 2])  # Plot positive frequencies
    plt.title("FFT of the Waveform")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.grid()
    plt.tight_layout()
    plt.show()
