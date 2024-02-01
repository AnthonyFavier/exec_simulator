import matplotlib.pyplot as plt
import numpy as np

Mean_HF =       np.array([ 6.2, 5.1, 5.2, 6.1, 5.2, 5.1, 6.1, 5.0, 6.1, 4.9, 6.1, 6.0 ])
SD_HF =         np.array([ 0.8, 1.1, 1.8, 1.0, 0.9, 1.0, 1.1, 1.3, 1.1, 1.3, 1.2, 1.1 ])
Mean_RF =       np.array([ 5.8, 3.6, 4.2, 4.6, 4.0, 3.8, 4.0, 3.8, 4.2, 2.9, 3.9, 4.3 ])
SD_RF =         np.array([ 1.3, 2.0, 1.9, 1.9, 1.9, 1.9, 2.0, 1.8, 2.0, 2.1, 2.2, 1.9 ])
Mean_Total =    np.array([ 6.0, 4.3, 4.7, 5.4, 4.6, 4.4, 5.1, 4.4, 5.2, 3.9, 5.0, 5.2 ])
SD_Total =      np.array([ 1.1, 1.7, 1.9, 1.7, 1.6, 1.7, 1.9, 1.7, 1.9, 2.0, 2.1, 1.8 ])


x = np.arange(1, len(Mean_HF)+1, 1)
plt.xticks(x)
plt.xlim(1, 12)
plt.ylim(1, 7)

plt.plot(x, Mean_HF, 'b-', label='mean_hf', marker='s')
plt.fill_between(x,  Mean_HF - SD_HF, Mean_HF + SD_HF, color='b', alpha=0.2)


plt.plot(x, Mean_RF, 'r-', label='mean_rf', marker='s')
plt.fill_between(x,  Mean_RF - SD_RF, Mean_RF + SD_RF, color='r', alpha=0.2)

plt.plot(x, Mean_Total, 'y--', label='mean_total', marker='s')

plt.grid()
plt.legend()
plt.show()