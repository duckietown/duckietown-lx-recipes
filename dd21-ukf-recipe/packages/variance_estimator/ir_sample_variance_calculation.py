#!/usr/bin/env python

import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys

def main(filepath = 'ir_data.txt'):
    ################################################################################
    ## Argument parser
    # def check_positive_interval_num(val):
    #     '''
    #     Function to check that the --num-intervals command-line argument is a positive
    #     float.
    #     '''
    #     value = int(val)
    #     if value <= 0:
    #         raise argparse.ArgumentTypeError('number of intervals must be positive')
    #     return value

    # parser = argparse.ArgumentParser(description=('Compute sample variance of IR '
    #                             'sensor and plot a histogram of sensor values.'))
    # parser.add_argument('--num-intervals', '-n', default=100, type=check_positive_interval_num,
    #                     help='Number of intervals to use in the histogram')
    # parser.add_argument('--line-chart', '-l', action='store_true',
    #         help=('Plot histogram as a line chart instead of a bar chart'))
    # args = parser.parse_args()
    num_intervals = 100 # args.num_intervals
    is_line_chart = True #args.line_chart

    ################################################################################
    # Load the range data from a txt file into a list
    ir_range_list = []
    with open(filepath, 'r') as f:
        while True:
            new_line = f.readline()
            if new_line == '':
                # Reached end of file (EOF), so break out of loop
                break
            if new_line != '---\n':
                # Got data, append to list
                ir_range_list.append(float(new_line))

    ################################################################################
    # Compute the sample variance
    num_samples = len(ir_range_list)
    mean = sum(ir_range_list)/float(num_samples)
    diffs_squared = [(x-mean)**2.0 for x in ir_range_list]
    sample_variance = sum(diffs_squared)/(num_samples-1)
    print('Sample variance:', sample_variance, 'm^2')

    ################################################################################
    # Prepare for plotting
    sample_std_dev = sample_variance**0.5

    lowest_ir_val = min(ir_range_list)
    highest_ir_val = max(ir_range_list)

    histogram_intervals = np.linspace(lowest_ir_val, highest_ir_val, num=num_intervals)
    histogram_frequencies = np.zeros(len(histogram_intervals))
    if not is_line_chart:
        # Determine width of bars
        if len(histogram_intervals) > 1:
            interval_size = (histogram_intervals[1] - histogram_intervals[0])*0.8
        else:
            # Use a default interval size
            interval_size = 0.1

    for num, ir in enumerate(ir_range_list):
        print_str = '\rSample points processed: {} / {}'.format(num, num_samples)
        interval_found = False
        interval = 0
        while not interval_found:
            # Cover edge case
            if interval + 1 == len(histogram_intervals):
                histogram_frequencies[interval] += 1
                interval_found = True
            elif ir >= histogram_intervals[interval] and ir < histogram_intervals[interval+1]:
                interval_found = True
                histogram_frequencies[interval] += 1
            else:
                interval += 1
    print_str = '\rSample points processed for plotting: {} / {}'.format(num_samples, num_samples)

    ################################################################################
    # Plot histogram of sample readings
    if is_line_chart:
        plt.plot(histogram_intervals, histogram_frequencies, linewidth=0.5)
    else:
        plt.bar(histogram_intervals, histogram_frequencies, width=interval_size)
    # plt.scatter(histogram_intervals, histogram_frequencies)
    plt.axvline(x=mean, color='k', linestyle='--', linewidth=0.8)
    plt.text(mean-0.003, 0.8*max(histogram_frequencies), 'Mean $\mu$: {0:1.5g}'.format(mean), rotation=90)
    plt.axvline(x=mean+sample_std_dev, color='red', linestyle='--', linewidth=0.8)
    plt.text(mean+sample_std_dev+0.0003, 0.01*max(histogram_frequencies), r'$\mu + \sigma$')
    plt.axvline(x=mean-sample_std_dev, color='red', linestyle='--', linewidth=0.8)
    plt.text(mean-sample_std_dev+0.0003, 0.01*max(histogram_frequencies), r'$\mu - \sigma$')
    plt.text(mean+0.0003, 0.01*max(histogram_frequencies), r'$\mu$')
    plt.ylim(ymin=0)
    plt.xlabel('ToF range reading (meters)')
    plt.ylabel('Number of samples')
    plt.show()
