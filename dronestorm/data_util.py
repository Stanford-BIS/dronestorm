"""Defines utilities for processing data"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve
from scipy.io.wavfile import read as wav_read
from scipy.io.wavfile import write as wav_write

AUDIO_SAMPLE_RATE = 44100 # standard 44.1kHz recording rate
AUDIO_SAMPLE_DT = 1./AUDIO_SAMPLE_RATE

def spike_wav(
        fname_spikes, fname_wav, nrn_idx=None,
        time_mode = "real",
        fname_spike_kernel=None, wav_dtype=np.int32,
        plot=False):
    """Create a wav audio file from the spike data

    Parameters
    ----------
    fname_spikes: string
        input spike data text filename
        first column is time
        subsequent columns are each neuron's spikes as would be recorded in the nengo simulator
    fname_wav: string
        filename of output wav data
    nrn_idx: list-like or none
        indices of neurons to use to generate wav file
        if None, uses all neurons, one per wav file channel
    time_mode: "real" or "sim"
        Whether to use the spike's real or simulation time
    fname_spike_kernel: string or None
        if not None, wav file name of kernel to convolve spikes with to generate different sounds
    wav_dtype: numpy dtype
        data type to be used in the wav file
    """
    assert isinstance(fname_spikes, str)
    assert isinstance(fname_wav, str)
    assert time_mode in ["real", "sim"]

    file_data = np.loadtxt(fname_spikes)
    sim_time = file_data[:, 0]
    measured_time = file_data[:, 1]
    spk_data_raw = file_data[:, 2:]
    if nrn_idx is not None:
        spk_data_raw = spk_data_raw[:, nrn_idx]
    n_samples, n_neurons = spk_data_raw.shape

    if time_mode == "real":
        time = measured_time
    elif time_mode == "sim":
        time = sim_time
    n_resamples = int(np.ceil(time[-1]*AUDIO_SAMPLE_RATE))
    spk_data = np.zeros((n_resamples, n_neurons))

    for nrn_idx in range(n_neurons):
        spk_idx = np.nonzero(spk_data_raw[:, nrn_idx])[0]
        spk_times = time[spk_idx]
        spk_resampled_idx = np.round(spk_times * AUDIO_SAMPLE_RATE).astype(int)
        spk_data[spk_resampled_idx, nrn_idx] = AUDIO_SAMPLE_RATE # impulse as 1/dt

    if fname_spike_kernel is not None:
        assert isinstance(fname_spike_kernel, str)
        spike_kernel_sample_rate, spk_kernel_data = wav_read(fname_spike_kernel)
        assert spike_kernel_sample_rate == AUDIO_SAMPLE_RATE, (
            "spike_kernel wav file sample rate must match AUDIO_SAMPLE_RATE")
        assert len(spk_kernel_data.shape) == 1, "spike_kernel wav data must have a single channel"
        spk_kernel_data = spk_kernel_data
        shift_idx = np.argmax(np.abs(spk_kernel_data)) # find peak of kernel for later alignment
        spk_data_preconv = spk_data.copy()
        spk_data_postconv = np.zeros((spk_data.shape[0]+len(spk_kernel_data)-1, n_neurons))

        for nrn_idx in range(n_neurons):
            spk_data_postconv[:, nrn_idx] = convolve(
                spk_data[:, nrn_idx], spk_kernel_data, mode="full")
        spk_data = spk_data_postconv
        spk_data = spk_data[shift_idx:] # align with original waveform
        spk_data = spk_data[:n_resamples] # clip to original length
    np.clip(spk_data, np.iinfo(wav_dtype).min, np.iinfo(wav_dtype).max, spk_data)
    spk_data = spk_data.astype(wav_dtype)
    wav_write(fname_wav, AUDIO_SAMPLE_RATE, spk_data)

    if plot:
        spk_data_fig = plt.figure()
        time_resampled = np.arange(n_resamples)*AUDIO_SAMPLE_DT
        if fname_spike_kernel is not None:
            ax = spk_data_fig.add_subplot(211)
            ax.plot(time_resampled, spk_data_preconv)
            ax.set_ylabel("raw spike waveform")
            ax.set_title("spike waveforms")
            ax = spk_data_fig.add_subplot(212, sharex=ax)
            ax.plot(time_resampled, spk_data)
            ax.set_ylabel("filtered spike waveform")
            ax.set_xlabel("time (s)")

            time_kernel = np.arange(len(spk_kernel_data))*AUDIO_SAMPLE_DT
            spk_kernel_fig = plt.figure()
            ax = spk_kernel_fig.add_subplot(111)
            ax.plot(time_kernel, spk_kernel_data)
            ax.set_title("spike kernel waveform")
            ax.set_xlabel("time (s)")
        else:
            ax = spk_data_fig.add_subplot(111)
            ax.plot(spk_data)
        plt.show()

def plot_tuning(fname_tuning_data, fname_plot_out=None, **kwargs):
    """Plot the tuning data

    Parameters
    ----------
    fname_tuning_data: string
        input tuning data filename
        first column is the input stimulus used to generate the tuning data
        remaining columns are the tuning data
    show: boolean
        whether or not to show the plot
    fname_plot_out: string or None
        filename to save the plot to if not None
        if None, shows the plot live
    Remaining keywords will be passed along to matplotlib for plotting
    """
    file_data = np.loadtxt(fname_tuning_data)
    stim = file_data[:, 0]
    tuning = file_data[:, 1:]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(stim, tuning, **kwargs)
    ax.set_xlabel('Input')
    ax.set_ylabel('Spike Rate (Hz)')

    if fname_plot_out is not None:
        assert isinstance(fname_plot_out, str)
        plt.savefig(fname_plot_out)
    else:
        plt.show()
