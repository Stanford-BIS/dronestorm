"""Module to communicate over PWM signals

Currently only reads PWM signals, but can extend to write PWM signals if
desired
"""
from __future__ import division
from __future__ import print_function
import pigpio

class PWMReader:
    """A class to read PWM pulses and calculate their frequency and duty cycle
    
    The frequency is how often the pulse happens per second
    The duty cycle is the percentage of pulse high time per cycle
    """
    def __init__(self, rpi, gpio, weighting=0.0):
        """
        Instantiate with the Pi and gpio of the PWM signal
        to monitor.

        Optionally a weighting may be specified.  This is a number
        between 0 and 1 and indicates how much the old reading
        affects the new reading.  It defaults to 0 which means
        the old reading has no effect.  This may be used to
        smooth the data.
        """
        self.rpi = rpi
        self.gpio = gpio

        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99

        self._new = 1.0 - weighting # Weighting for new reading.
        self._old = weighting       # Weighting for old reading.

        self._high_tick = None
        self._period = None
        self._high = None

        rpi.set_mode(gpio, pigpio.INPUT)

        self._cb = rpi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):
        """Callback function to trigger when level changes"""
        if level == 1:
            if self._high_tick is not None:
                t_diff = pigpio.tickDiff(self._high_tick, tick)
                if self._period is not None:
                    self._period = self._old*self._period + self._new*t_diff
                else:
                    self._period = t_diff
            self._high_tick = tick
        elif level == 0:
            if self._high_tick is not None:
                t_diff = pigpio.tickDiff(self._high_tick, tick)
                if self._high is not None:
                    self._high = self._old*self._high + self._new*t_diff
                else:
                    self._high = t_diff

    def frequency(self):
        """
        Returns the PWM frequency.
        """
        if self._period is not None:
            return 1000000.0 / self._period
        else:
            return 0.0

    def pulse_width(self):
        """
        Returns the PWM pulse width in microseconds.
        """
        if self._high is not None:
            return self._high / 1000000
        else:
            return 0.0

    def duty_cycle(self):
        """
        Returns the PWM duty cycle percentage.
        """
        if self._high is not None:
            return 100.0 * self._high / self._period
        else:
            return 0.0

    def cancel(self):
        """
        Cancels the reader and releases resources.
        """
        self._cb.cancel()
