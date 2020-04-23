############################################
##reference: https://www.machinelearningplus.com/plots/matplotlib-tutorial-complete-guide-python-plot-examples/#2.-A-Basic-Scatterplot
#############################################

import matplotlib.pyplot as plt

def plot_number_list():
    # Plot a list of numbers will get a line
    plt.plot([1, 2, 3, 4, 10])

def plot_scatter_dots():
    # The argument of plt is [x, y, format], and the format is {color}{marker}{line}
    # plt.figure(figsize=(10,7)) # 10 is width, 7 is height
    plt.plot([1,2,3,4,5], [1,2,3,4,10], 'go', label='GreenDots')
    plt.plot([1,2,3,4,5], [2,3,4,5,11], 'b*', label='Bluestars')
    plt.title('A Simple Scatterplot')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc='best')  # legend text comes from the plot's label parameter.
    #plt.savefig('figure.svg', format='svg')

def plot_scatter_in_subplot():
    # Create Figure and Subplots
    fig, (ax1, ax2) = plt.subplots(1,2, figsize=(10,4), sharey=True, dpi=120)

    # Plot
    ax1.plot([1,2,3,4,5], [1,2,3,4,10], 'go')  # greendots
    ax2.plot([1,2,3,4,5], [2,3,4,5,11], 'b*')  # bluestart

    # Title, X and Y labels, X and Y Lim
    ax1.set_title('Scatterplot Greendots'); ax2.set_title('Scatterplot Bluestars')
    ax1.set_xlabel('X');  ax2.set_xlabel('X')  # x label
    ax1.set_ylabel('Y');  ax2.set_ylabel('Y')  # y label
    ax1.set_xlim(0, 6) ;  ax2.set_xlim(0, 6)   # x axis limits
    ax1.set_ylim(0, 12);  ax2.set_ylim(0, 12)  # y axis limits

    ## Important!!
    ## Above codes can be replaced with the following two line codes
    # ax1.set(title='Scatterplot Greendots', xlabel='X', ylabel='Y', xlim=(0,6), ylim=(0,12))
    # ax2.set(title='Scatterplot Bluestars', xlabel='X', ylabel='Y', xlim=(0,6), ylim=(0,12))

    # ax2.yaxis.set_ticks_position('none') 
    plt.tight_layout()

def plot_scatter_in_subplot2():
    ## The matlab-syntax coding
    # Left hand side plot
    plt.subplot(1,2,1)  # (nRows, nColumns, axes number to plot)
    plt.plot([1,2,3,4,5], [1,2,3,4,10], 'go')  # green dots
    plt.title('Scatterplot Greendots')  
    plt.xlabel('X'); plt.ylabel('Y')
    plt.xlim(0, 6); plt.ylim(0, 12)

    # Right hand side plot
    plt.subplot(1,2,2)
    plt.plot([1,2,3,4,5], [2,3,4,5,11], 'b*')  # blue stars
    plt.title('Scatterplot Bluestars')  
    plt.xlabel('X'); plt.ylabel('Y')
    plt.xlim(0, 6); plt.ylim(0, 12)

def plot_use_axes():
    # Draw multiple plots using for-loops using object oriented syntax
    import numpy as np
    from numpy.random import seed, randint
    seed(100)

    # Create Figure and Subplots
    fig, axes = plt.subplots(2,2, figsize=(10,6), sharex=True, sharey=True, dpi=120)

    # Define the colors and markers to use
    colors = {0:'g', 1:'b', 2:'r', 3:'y'}
    markers = {0:'o', 1:'x', 2:'*', 3:'p'}

    # Plot each axes
    for i, ax in enumerate(axes.ravel()):
        ax.plot(sorted(randint(0,10,10)), sorted(randint(0,10,10)), marker=markers[i], color=colors[i])  
        ax.set_title('Ax: ' + str(i))
        ax.yaxis.set_ticks_position('none')

    plt.suptitle('Four Subplots in One Figure', verticalalignment='bottom', fontsize=16)    
    plt.tight_layout()

if __name__ == '__main__':
    # plt.grid(linestyle="-.") # Set the grid
    # plot_scatter_dots()
    plot_use_axes()
    plt.show()