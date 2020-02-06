# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)



# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.
def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Put your code here.
    mult_start = min(a.start(), b.start())
    mult_stop = max(a.stop(), b.stop())
    mult_values = []
    for i in range(mult_start, mult_stop):
        mult_values.append(a.value(i)*b.value(i))
    mult_dist = Distribution(mult_start, mult_values)
    mult_dist.normalize()
    return mult_dist

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    dist_list = []
    offset = a.offset + b.offset  # this command works to displace the distribution by the given 'move'
    for a_value in a.values:
        mini_dist_val = []
        for b_value in b.values:
            mini_dist_val.append(a_value*b_value)
        dist_list.append(Distribution(offset, mini_dist_val)) # List of all mini distributions
        offset += 1    # offset for the next small distribution
    a_conv = Distribution.sum(dist_list)
    return a_conv


if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
