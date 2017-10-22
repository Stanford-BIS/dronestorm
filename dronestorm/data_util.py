"""Defines utilities for processing data"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.signal import convolve
from scipy.io.wavfile import read as wav_read
from scipy.io.wavfile import write as wav_write

AUDIO_SAMPLE_RATE = 44100 # standard 44.1kHz recording rate
AUDIO_SAMPLE_DT = 1./AUDIO_SAMPLE_RATE

def _load_spike_data(fname_spikes):
    """Utility function to load in spike data

    Parameters
    ----------
    fname_spikes: string
        input spike data text filename
        first column is sim time
        second column is real time
        subsequent columns are each neuron's spikes as would be recorded in the nengo simulator
    """
    file_data = np.loadtxt(fname_spikes)
    sim_time = file_data[:, 0]
    real_time = file_data[:, 1]
    spk_data_raw = file_data[:, 2:]
    return sim_time, real_time, spk_data_raw

def _filter_nrn_idx_yticks(yticks, n_neurons):
    """Removes non-integer and out-of-range ticks"""
    tol = 0.001
    ret_ticks = []
    for ytick in yticks:
        ret_tick = int(np.round(ytick))
        if abs(ret_tick - ytick) < tol and ytick >=0 and ytick < n_neurons:
            ret_ticks += [ret_tick]
    return ret_ticks

def animate_spike_raster(
        fname_spikes, fname_movie_out, nrn_idx=None, time_mode="real",
        time_stop=None, time_window=1.0, fps=30):
    """Generate a movie from the spike raster

    Parameters
    ----------
    fname_spikes: string
        input spike data text filename
        first column is sim time
        second column is real time
        subsequent columns are each neuron's spikes as would be recorded in the nengo simulator
    fname_movie_out: string
        if string, filename of output movie
    nrn_idx: list-like or none
        indices of neurons to use to generate wav file
        if None, uses all neurons, one per wav file channel
    time_mode: "real" or "sim"
        Whether to use the spike's real or simulation time
    time_stop: float or None
        if float, movie ends time_window after time_stop
        if None, movie ends after time_window past last spike
    time_window: float
        Size of time window over which to view spikes
    fps: int
        frames per second
    """
    sim_time, real_time, spk_data_raw = _load_spike_data(fname_spikes)
    if nrn_idx is not None:
        spk_data_raw = spk_data_raw[:, nrn_idx]
    n_samples, n_neurons = spk_data_raw.shape
    if time_mode == "real":
        time = real_time
    elif time_mode == "sim":
        time = sim_time
    spk_times = []
    for nrn_idx in range(n_neurons):
        spk_idx = np.nonzero(spk_data_raw[:, nrn_idx])[0]
        spk_times.append(time[spk_idx])

    fig = plt.figure()

    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    ax = fig.add_subplot(111)
    y_range = 0.8
    last_spk_time = 0.
    for nrn_idx in range(n_neurons):
        y_mid = nrn_idx
        y_min = y_mid - y_range/2.
        y_max = y_mid + y_range/2.
        spk_idx = np.nonzero(spk_data_raw[:, nrn_idx])[0]
        spk_times = time[spk_idx]
        last_spk_time = np.max(spk_times.tolist() + [last_spk_time])
        ax.vlines(spk_times, y_min, y_max, colors[nrn_idx%len(colors)])
    ax.set_ylim(-y_range/1.5, n_neurons-1+y_range/1.5)
    ax.set_title("Spike Raster")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Neuron Index")
    ax.set_yticks(_filter_nrn_idx_yticks(ax.get_yticks(), n_neurons))

    time_start = -time_window
    if time_stop is None:
        time_stop = last_spk_time+time_window
    else:
        time_stop = time_stop+time_window
    animation_time_total = time_stop - time_start - time_window
    animation_dt = 1./fps
    n_frames = int(np.round(animation_time_total*fps))

    moviewriter = animation.FFMpegWriter(fps=fps)
    with moviewriter.saving(fig, fname_movie_out, dpi=100):
        for frame_idx in range(n_frames):
            time_since_start = frame_idx*animation_dt
            ax.set_xlim(time_start+time_since_start, time_start+time_since_start+time_window)
            moviewriter.grab_frame()

def animate_tuning(
        fname_tuning_data, fname_input_data, fname_movie_out, input_data_col=2,
        time_mode="real", fps=30,
        label=True, xlabel="Input"):
    """Generate a movie from the spike raster

    Parameters
    ----------
    fname_tuning: string
        tuning data filename
        first column is the input stimulus used to generate the tuning data
        remaining columns are the tuning data
    fname_input_data: string
        input data filename
        first column is sim time
        first column is real time
        subsequent columns are each inputs dimension
    fname_movie_out: string
        if string, filename of output movie
    nrn_idx: list-like or none
        indices of neurons to use to generate wav file
        if None, uses all neurons, one per wav file channel
    time_mode: "real" or "sim"
        Whether to use the spike's real or simulation time
    fps: int
        frames per second
    label: boolean
        whether or not to label the tuning curves and display a legend
    xlabel: string
        x axis label
    """
    fig, ax = plot_tuning(fname_tuning_data, show=False, label=label, xlabel=xlabel)
    ylim = ax.get_ylim()

    file_data = np.loadtxt(fname_input_data)
    sim_time = file_data[:, 0]
    real_time = file_data[:, 1]
    input_data = file_data[:, input_data_col]

    if time_mode == "real":
        time = real_time
    elif time_mode == "sim":
        time = sim_time

    n_frames = int(np.round(time[-1]*fps))
    time_idx = 0
    len_time = len(time)
    in_dat_line = ax.plot(
        [input_data[time_idx], input_data[time_idx]],
        [ylim[0], ylim[1]], 'r:')[0]
    ax.set_ylim(ylim)

    moviewriter = animation.FFMpegWriter(fps=fps)
    with moviewriter.saving(fig, fname_movie_out, dpi=100):
        for frame_idx in range(n_frames):
            curr_time = frame_idx / fps
            moved = False
            while time[time_idx] <= curr_time:
                time_idx += 1
                moved = True
            if moved:
                time_idx -= 1
            if time_idx < len_time:
                in_dat_line.set_data(
                    [input_data[time_idx], input_data[time_idx]], [ylim[0], ylim[1]])
                moviewriter.grab_frame()

def make_spike_wav(
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
        if string, wav file name of kernel to convolve spikes with to generate different sounds
    wav_dtype: numpy dtype
        data type to be used in the wav file
    """
    assert isinstance(fname_spikes, str)
    assert isinstance(fname_wav, str)
    assert time_mode in ["real", "sim"]

    sim_time, measured_time, spk_data_raw = _load_spike_data(fname_spikes)
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

def plot_timing(fname_spikes, fname_plot_out=None):
    """Plot the simulation timing data"""
    file_data = np.loadtxt('spikes.txt')
    sim_time = file_data[:, 0]
    measured_time = file_data[:, 1]

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(211)
    ax.plot(measured_time, label="measured")
    ax.plot(sim_time, label="simulation")
    ax.legend(loc="lower right")
    ax.set_ylabel('cumulative time (s)')
    ax.set_title("Simulation Timing")
    ax = fig.add_subplot(212, sharex=ax)
    ax.plot(np.diff(measured_time), label="measured")
    ax.plot(np.diff(sim_time), label="simulation")
    ax.legend(loc="upper right")
    ax.set_ylabel('delta t (s)')
    ax.set_xlabel('Simulation step index')
    if fname_plot_out is not None:
        assert isinstance(fname_plot_out, str)
        plt.savefig(fname_plot_out)
    else:
        plt.show()

def plot_tuning(
        fname_tuning_data, fname_plot_out=None, show=True, label=False, xlabel="Input", **kwargs):
    """Plot the tuning data

    Returns the figure and axis handle of the plot

    Parameters
    ----------
    fname_tuning_data: string
        input tuning data filename
        first column is the input stimulus used to generate the tuning data
        remaining columns are the tuning data
    show: boolean
        whether or not to show the plot
    label: boolean
        whether or not to label the tuning curves and display a legend
    xlabel: string
        x axis label
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
    if label:
        for nrn_idx in range(tuning.shape[1]):
            ax.plot(stim, tuning[:, nrn_idx], label="Neuron %d"%nrn_idx, **kwargs)
        ax.legend(loc="best")
    else:
        ax.plot(stim, tuning, **kwargs)
    ax.set_xlabel(xlabel)
    ax.set_ylabel('Spike Rate (Hz)')
    ax.set_title("Tuning Curves")
    if fname_plot_out is not None:
        assert isinstance(fname_plot_out, str)
        plt.savefig(fname_plot_out)
    if show:
        plt.show()

    return fig, ax
