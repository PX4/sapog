#!/usr/bin/env python
#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#
# ESC dataset plotter
#

import sys
from functools import partial
from pylab import *
from scipy import signal

LOW_RPM = 900

# -----------------

def read_csv_to_column_dict(filename):
    with open(filename) as f:
        data = {}
        for index,line in enumerate(f):
            if index == 0:
                column_names = line.split()
            else:
                for name, value in zip(column_names, line.split()):
                    data.setdefault(name, []).append(float(value))
    for key, value in data.items():
        data[key] = array(value)
    assert set(data.keys()) == set(column_names)
    assert len(set(map(len, data.values()))) == 1
    return data

def strip_data_with_zero_values(in_data, column_name):
    keep_indices = map(lambda x: x > 0.0, in_data[column_name])
    out_data = {}
    for key, value in in_data.items():
        out_data[key] = array(v for i, v in enumerate(value) if keep_indices[i])
    assert len(set(map(len, out_data.values()))) == 1
    return out_data

def prepare_input_data(filename):
    print 'Reading', filename
    data = read_csv_to_column_dict(filename)
    print 'Raw data points:', len(data.values()[0])
    data = strip_data_with_zero_values(data, 'rpm')
    print 'Stripped data points:', len(data.values()[0])
    print 'Max current:', max(data['current'])

    data['power'] = data['voltage'] * data['current']

    fir = signal.firwin(numtaps=10, cutoff=300.0, nyq=1000.0)
    lpf = partial(signal.lfilter, fir, 1)

    data['power_lpf'] = lpf(data['power'])
    data['current_lpf'] = lpf(data['current'])
    data['voltage_lpf'] = lpf(data['voltage'])
    return data

def add_dynamic_plots(data, axes):
    # TODO: multiplots
    def makeplot(ax, y1, y2=None):
        x = data['time']
        def plot_one(ax, y, yscatter, ylabel, color):
            ax.plot(x, y, color + '-')
            if yscatter is not None:
                ax.scatter(x, yscatter, color=color, marker='.', alpha=0.2)
            ax.set_ylabel(ylabel, color=color)
            for tl in ax.get_yticklabels():
                tl.set_color(color)
            ax.xaxis.set_ticks(range(int(x[0]), int(x[-1]) + 1))
            ax.set_xlim([x[0], x[-1]])
            ax.grid(color=color, axis='y', alpha=GRID_ALPHA)
            ax.grid(axis='x', alpha=GRID_ALPHA)

        plot_one(ax, y1['data'], y1.get('scatter'), y1['label'], y1['color'])
        ax.set_xlabel('Time (s)')
        if y2:
            ax2 = ax.twinx()
            plot_one(ax2, y2['data'], y2.get('scatter'), y2['label'], y2['color'])

    y_rpm = {
        'data': data['rpm'],
        'label': 'RPM',
        'color': 'b'
    }
    y_power = {
        'data': data['power_lpf'],
        'scatter': data['power'],
        'label': 'Power (W)',
        'color': 'r'
    }
    y_dc = {
        'data': data['dc'] * 100.0,
        'label': 'Duty cycle (%)',
        'color': 'g'
    }
    makeplot(axes[0], y_rpm, y_power)
    makeplot(axes[1], y_dc, y_rpm)
    makeplot(axes[2], y_power, y_dc)

def add_efficiency_plots(data, ax, color, label):
    data = strip_data_with_zero_values(data, 'power')
    print 'Stripped zero power:', len(data.values()[0])

    x = data['rpm']
    y = data['power']

    #ax.scatter(x, y, marker=',', color=color, s=0.1, alpha=0.1)
    ax.set_xlabel('RPM')
    ax.set_ylabel('Power (W)')

    a, b, c, d = np.polyfit(x, y, 3)
    fitted = a * (x ** 3) + b * (x ** 2) + c * x + d
    ax.plot(x, fitted, color, label=label)
    ax.grid(alpha=GRID_ALPHA)

def add_ripple_plots(data, axes, color, label, field, plot_name):
    data = data.copy()
    data['dc_raising'] = [True] + map(lambda i: data['dc'][i] > data['dc'][i - 1], xrange(1, len(data['dc'])))
    data = strip_data_with_zero_values(data, 'dc_raising')
    print 'Stripped raising DC:', len(data.values()[0])
    data['low_rpm'] = map(lambda x: x > LOW_RPM, data['rpm'])
    data = strip_data_with_zero_values(data, 'low_rpm')
    print 'Stripped low RPM:', len(data.values()[0])

    ax_time, ax_ripple = axes
    time = data['time']
    y = data[field]

    ax_time.scatter(time, y, marker=',', color=color, s=0.1, alpha=0.5)
    ax_time.set_xlabel('Time (S)')
    ax_time.set_ylabel(plot_name)

    poly_y = np.polyfit(time, y, 11)
    fitted_y = np.polyval(poly_y, time)
    ax_time.plot(time, fitted_y, color, label=label)
    ax_time.grid(alpha=GRID_ALPHA)

    ripple = map(abs, fitted_y - y)
    #ax_ripple.scatter(y, ripple, marker=',', color=color, s=0.1, alpha=0.1)
    ax_ripple.set_xlabel(plot_name)
    ax_ripple.set_ylabel(plot_name + ' Ripple')

    poly_ripple = np.polyfit(y, ripple, 6)
    fitted_ripple = np.polyval(poly_ripple, y)
    ax_ripple.plot(y, fitted_ripple, color, label=label)
    ax_ripple.grid(alpha=GRID_ALPHA)

def plot_dynamic(filenames):
    fig, axes = subplots(3, sharex=True)
    fig.tight_layout()
    fig.subplots_adjust(hspace=0.1)
    for fn in filenames:
        data = prepare_input_data(fn)
        add_dynamic_plots(data, axes)
    return fig

def plot_efficiency(filenames):
    fig, ax = subplots()
    fig.tight_layout()
    for color,fn in zip(COLORS, filenames):
        data = prepare_input_data(fn)
        add_efficiency_plots(data, ax, color, fn)
    legend(loc=2)
    return fig

def plot_ripple(filenames):
    fig, ((ax1, ax2), (ax3, ax4)) = subplots(2, 2)
    fig.tight_layout()
    for color,fn in zip(COLORS, filenames):
        data = prepare_input_data(fn)
        add_ripple_plots(data, (ax1, ax2), color, fn, 'rpm', 'RPM')
        add_ripple_plots(data, (ax3, ax4), color, fn, 'power_lpf', 'Power')
    legend(loc=2)
    return fig

# -----------------

COLORS = ['b', 'r', 'g', 'y']

GRID_ALPHA = 0.4
mpl.rcParams['lines.linewidth'] = 1.0
mpl.rcParams['grid.linestyle'] = '-'
mpl.rcParams['font.size'] = 10

if len(sys.argv) < 3:
    print sys.argv[0], '<dynamic|efficiency|ripple> <CSV path [...]>'
    exit(1)
plot_type = sys.argv[1].lower()
filenames = sys.argv[2:]

if plot_type == 'dynamic':
    fig = plot_dynamic(filenames)
elif plot_type == 'efficiency':
    fig = plot_efficiency(filenames)
elif plot_type == 'ripple':
    fig = plot_ripple(filenames)
else:
    print 'Invalid plot type:', plot_type
    exit(1)

fig.set_size_inches(9, 7)
#savefig(plot_type + '.svg', bbox_inches='tight')
savefig(plot_type + '.png', bbox_inches='tight')
show()
