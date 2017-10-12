"""Defines utilities for processing data"""
from scipy.io.wavfile import write as wav_write

AUDIO_SAMPLE_RATE = 48000 # standard 48kHz recording rate
AUDIO_SAMPLE_DT = 1./AUDIO_SAMPLE_RATE

def spike_wav(fname_spikes, fname_wav, nrn_idx=None):
    """Create a wav audio file from the spike data
    
    Parameters
    ----------
    fname_spikes: string
        filename of input spike data
        spike data should be formatted as text file of rows and columns
        first column is time
        subsequent columns are each neuron's spikes as would be recorded in the nengo simulator
    fname_wav: string
        filename of output wav data
    nrn_idx: list-like or none
        indices of neurons to use to generate wav file
        if None, uses all neurons, one per wav file channel
    """
    assert isinstance(fname_spikes, str)
    assert isinstance(fname_wav, str)
    file_data = np.loadtxt(fname_spikes)
    time = file_data[:, 0]
    spk_data = file_data[:, 1:]

    n_samples, n_neurons = spk_data.shape
    dt = time[1] - time[0]
    
    n_resamples = time[-1]*AUDIO_SAMPLE_RATE
    spk_data_resampled = np.zeros((n_resamples, n_neurons))



    
    # resample sp
    wav_write(fname_wav, AUDIO_SAMPLE_RATE, p

def plot_tuning(fname_tuning_data, fname_plot_out=None):
    """Plot the tuning data
    
    Parameters
    ----------
    fname_tuning_data: string
    """
