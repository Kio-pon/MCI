#!/usr/bin/env python3
"""
ADC Simulator - Generates simulated ADC values from 0 to 4095
Mimics the STM32 ADC output format for testing
"""

import time
import random

def main():
    """Simulate ADC values ramping from 0 to 4095 with random step sizes"""
    
    print("Starting ADC simulation: 0 -> 4095 (with random step rates)")
    print("Format: ADC Value: <raw_value>")
    print("-" * 40)
    
    adc_raw = 0
    while adc_raw < 4096:
        # Output in same format as STM32 Task 1 code: "ADC Value: %lu\r\n"
        print(f"ADC Value: {min(adc_raw, 4095)}")
        
        # Random increment: 1 to 50 (variable rate of change)
        adc_raw += random.randint(1, 50)
        
        # Delay to mimic 10ms sampling rate (optional - comment out for fast generation)
        time.sleep(0.01)
    
    print("-" * 40)
    print("Simulation complete")

if __name__ == "__main__":
    main()
