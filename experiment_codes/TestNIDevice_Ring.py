
import nidaqmx
from time import time, sleep
from numpy import array, matmul
from numpy.fft import fft, ifft, fftshift, ifftshift
import numpy as np
import matplotlib.pyplot as plt
from nidaqmx.constants import Edge, AcquisitionType, LineGrouping
import threading
from time import sleep
import keyboard
from copy import copy

class NI_Device:

    dev_name = 'NI USB-6343'
    task = ''

    N_SAMPLES = 500
    FREQ = 1e6
    N_SENSORS = 20

    calibrations = [4.916815742,
                    7.875358166,
                    2.312578881,
                    2.406742557,
                    5.57505071,
                    4.086988848,
                    7.242424242,
                    1.143302829,
                    1.273928158,
                    4.096125186,
                    1.077420619,
                    1.,
                    1.,
                    2.528518859,
                    3.463768116,
                    10.27476636,
                    11.01603206,
                    5.558139535,
                    7.204456094,
                    4.788327526,
                    9.593368237,
                    3.548741123,
                    3.507977026,
                    1]

    def __init__(self, frequency=1e6, samples=900):

        self.FREQ = frequency
        self.N_SAMPLES = samples

        self.x_data = np.linspace(0, self.FREQ, self.N_SAMPLES)

        self.offsets = np.zeros(self.N_SENSORS)
        self.kForce = -.01

        self.last_samples = np.zeros(self.N_SENSORS)
        self.a = .2

        self.task = nidaqmx.Task()
        self.task.di_channels.add_di_chan('Dev1/port0/line0:23')
        self.task.timing.cfg_samp_clk_timing(self.FREQ, sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                                           source="", samps_per_chan=self.N_SAMPLES)

    def readSample(self, n):
        return self.task.read(number_of_samples_per_channel=1000)

    def readSingleData(self):
        return self.task.read()

    def getFFTData(self):
        data = [t.read(self.N_SAMPLES) for t in self.task]
        data = np.array(data)*1

        fft_data = [np.abs(fft(d)) for d in data]

        return fft_data

    def getMainFreq(self):
        data = self.task.read(self.N_SAMPLES)

        data_diff = np.array(data[1:]) ^ np.array(data[:-1])

        freq = []
        for i in range(self.N_SENSORS):
            data_tmp = data_diff & (0b1 << i)

            idx1 = np.nanargmax(data_tmp)
            idx2 = np.nanargmax(data_tmp[idx1 + 1:])
            idx3 = np.nanargmax(data_tmp[idx1 + idx2 + 2:])
            period = (idx3 + idx2) / self.FREQ
            #period = min(idx3, idx2) / self.FREQ * 2
            if period == 0:
                period = 1e15

            freq.append(1 / period)

        return np.array(freq)

    def calculateOffset(self):
        offsets = np.zeros(self.N_SENSORS)
        for i in range(20):
            data = self.getMainFreq()
            for j in range(self.N_SENSORS):
                offsets[j] += data[j] / 20.

        self.offsets = offsets
        self.last_samples = np.copy(self.offsets)

    def getForceMetric(self):

        data = self.last_samples * (1. - self.a) + self.a * np.array(self.getMainFreq())

        self.last_samples = copy(data)
        # for i in range(self.N_SENSORS):
        #     data[i] -= self.offsets[i]
        #     data[i] *= self.kForce

        data = (data - self.offsets) * self.kForce * self.calibrations[:self.N_SENSORS]

        return data



if __name__ == "__main__":
    NIdev = NI_Device()

    count = 1

    T0 = time()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    line1, = ax.plot([], [])


    NIdev.calculateOffset()

    def udatePlot(x, y):
        xmax = np.max(x)
        ymin = -40
        ymax = 40
        ymin = np.min(y)
        ymax = np.max(y)

        ydelta = ymax - ymin
        ymin -= ydelta * .1
        ymax += ydelta * .1

        ax.set_xlim([min(0, xmax - 15), xmax])
        ax.set_ylim([ymin, ymax])

        line1.set_xdata(x)
        line1.set_ydata(y)

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(.001)

    x = np.arange(1, 21)
    y_l = [[] for i in range(NIdev.N_SENSORS)]
    t0 = time()
    while (True):
        tmp = np.array(NIdev.getForceMetric())
        tmp = np.array(NIdev.getMainFreq())
        #NIdev.offsets[tmp>NIdev.offsets] = tmp[tmp>NIdev.offsets]
        #tmp = (tmp - NIdev.offsets) * NIdev.kForce
        print(tmp)
        udatePlot(x, tmp)

        # if keyboard.is_pressed('d'):
        #     NIdev.sptask.stop()
        #     NIdev.dirtask.write(True)
        #     NIdev.sptask.start()
        #
        # elif keyboard.is_pressed('a'):
        #     NIdev.sptask.start()
        #     NIdev.dirtask.stop(False)
        #     NIdev.sptask.start()
        #
        # elif keyboard.is_pressed('s'):
        #     NIdev.sptask.stop()
        #
        # print(tmp)


