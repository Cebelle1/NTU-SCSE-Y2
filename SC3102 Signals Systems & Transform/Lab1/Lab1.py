import matplotlib.pyplot as plt
import numpy as np
from scipy import signal as sgn


x =np.linspace(-12.5, 12.5, 101)
plt.figure()

y1 = np.sin(x)                  #Sine wave
y2 = np.cos(x)                  #Cosine wave
y3 = sgn.unit_impulse(101,[0])         #Unit Impulse Wave
y4 = x>0                        #Unit Step wave
y5 = (x>-3)*(x<3)               #Unit Square Wave
y6 = sgn.sawtooth(x, 0.5)       #Triangular Wave
y7 = np.exp(np.linspace(0,10, 101))                   #Exponential Wave
y8 = sgn.sawtooth(x, 0)             #Sawtooth Wave
y9 = np.sign(x)           #Sign function
y10 = np.sinc(x)                #Sinc wave

wave_name = ['Sine wave', ' Cosine wave', 'Unit Impulse wave', 'Unit step wave', 'Square wave',
             'Triangular wave', 'Exponential wave', 'Sawtooth wave', 'Sign function', 'Sinc function']

y = [y1,y2,y3,y4,y5,y6,y7,y8,y9,y10]

plt.figure(figsize = (10,10))

for i in range(10):
    plt.subplot(5,2,i+1)
    plt.plot(x,y[i])
    plt.title(wave_name[i])
    plt.ylabel('Amplitude')
    plt.xlabel('Time')
    
plt.tight_layout()
plt.show()

