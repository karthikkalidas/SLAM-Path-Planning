# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    # width_a = (Distribution.stop(a)-Distribution.start(a)-1)/2
    # width_b = (Distribution.stop(b)-Distribution.start(b)-1)/2
    # conv_offset = b.offset - width_a
    # conv_values = []
    # conv_value_i = 0
    # for j in range(2*width_b+1):
    #     for i in range(2*width_a+1):
    #         conv_value_ij += Distribution.value(a,a.offset + i)*Distribution.value(b,b.offset + j)
    #     conv_values.append(conv_value_ij)
    # # --->>> Put your code here.
    # c = Distribution(conv_offset,conv_values)
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
    arena = (0,100)

    # Move 3 times by 20.
    moves = [20] * 3

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Now move and plot.
    for m in moves:
        move_distribution = Distribution.triangle(m, 2)
        position = convolve(position, move_distribution)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             linestyle='steps')

    ylim(0.0, 1.1)
    show()
