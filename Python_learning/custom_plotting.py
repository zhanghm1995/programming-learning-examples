############################################
##reference: https://www.machinelearningplus.com/plots/matplotlib-tutorial-complete-guide-python-plot-examples/#2.-A-Basic-Scatterplot
## Learn how to customise the legend, title, texts and so on
#############################################

import matplotlib.pyplot as plt
import numpy as np

def customise_legend():
    # plt.style.use('seaborn-notebook')
    plt.figure(figsize=(10,7), dpi=80)
    X = np.linspace(0, 2*np.pi, 1000)
    sine = plt.plot(X,np.sin(X)); cosine = plt.plot(X,np.cos(X))
    sine_2 = plt.plot(X,np.sin(X+.5)); cosine_2 = plt.plot(X,np.cos(X+.5))
    plt.gca().set(ylim=(-1.25, 1.5), xlim=(-.5, 7))
    plt.title('Custom Legend Example', fontsize=18)
    # Modify legend
    plt.legend([sine[0], cosine[0], sine_2[0], cosine_2[0]],   # plot items
            ['sine curve', 'cosine curve', 'sine curve 2', 'cosine curve 2'],  
            frameon=True,                                   # legend border
            framealpha=1,                                   # transparency of border
            ncol=2,                                         # num columns
            shadow=True,                                    # shadow on
            borderpad=1,                                    # thickness of border
            title='Sines and Cosines')                      # title

if __name__ == '__main__':
    customise_legend()
    plt.show()