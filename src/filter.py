#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  3 12:39:28 2025

@author: mike
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def second_order_filter(b, a, x):
    """
    Applies a second-order linear discrete filter (direct form II transposed).
    Assumes a[0] is normalized to 1.
    
    Args:
        b (list or array): Numerator coefficients [b0, b1, b2].
        a (list or array): Denominator coefficients [a0, a1, a2] (a0=1).
        x (array): Input signal.

    Returns:
        array: Filtered output signal.
    """
    if a[0] != 1.0:
        # Normalize coefficients if a[0] is not 1
        b = np.array(b) / a[0]
        a = np.array(a) / a[0]
    
    y = np.zeros_like(x)
    # Filter states (d1, d2)
    d1 = 0.0
    d2 = 0.0
    
    # Difference equations for Direct Form II Transposed (second order)
    # y[n] = b0*x[n] + d1[n-1]
    # d1[n] = b1*x[n] - a1*y[n] + d2[n-1]
    # d2[n] = b2*x[n] - a2*y[n]
    
    for n in range(len(x)):
        y[n] = b[0] * x[n] + d1
        d1 = b[1] * x[n] - a[1] * y[n] + d2
        d2 = b[2] * x[n] - a[2] * y[n]
        
    return y

# Example usage with same coefficients as above:
# b, a = signal.butter(2, 0.1, btype='low', analog=False) # Get the coefficients first
# y_custom = second_order_filter(b, a, x)



# 1. Define filter coefficients (b and a)
# Example: A second-order Butterworth low-pass filter with a normalized cutoff frequency of 0.2
# The 'butter' function returns the coefficients b and a.
# Wn is a fraction of the Nyquist frequency (half the sampling frequency).
b, a = signal.butter(2, 0.2, 'low', analog=False)
print(f"Numerator coefficients (b): {b}")
print(f"Denominator coefficients (a): {a}")

# 2. Generate a sample input signal (e.g., a noisy signal with a low-frequency component)
fs = 1000  # Sampling frequency
t = np.linspace(0, 1, fs, endpoint=False)  # 1 second duration
# Signal with low freq (10 Hz) and high freq (200 Hz) components
x = np.sin(2 * np.pi * 10 * t) + 0.0 * np.sin(2 * np.pi * 200 * t) + np.random.randn(fs) * 0.0

# 3. Apply the filter to the input signal
y = second_order_filter(b, a, x)
# for n in range(len(t)):
#     print(t[n], ",", y[n])

# 4. Plot the results
plt.figure(figsize=(10, 6))
plt.plot(t, x, label='Original signal', alpha=0.5)
plt.plot(t, y, label='Filtered signal', color='red')
plt.title('Second Order Linear Discrete Filter Example')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.legend()
plt.grid()
plt.show()
