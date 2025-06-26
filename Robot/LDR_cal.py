import machine
import utime

# Define the ADC pin
adc = machine.ADC(27)

# Function to read ADC value
def read_adc():
    return adc.read_u16()

# Function to map the ADC value to a light level percentage
def map_adc_to_light_level(adc_value, min_adc, max_adc):
    # Ensure the min_adc and max_adc are not equal to avoid division by zero
    if max_adc == min_adc:
        return 0
    # Calculate the light level percentage
    light_level = (max_adc - adc_value) / (max_adc - min_adc) * 100
    return light_level

# Calibration function
def calibrate_ldr(samples=100):
    min_val = 0
    max_val = 65535
    
    print("Calibrating... Please expose the LDR to the brightest and darkest conditions during this time.")
    for i in range(samples):
        adc_value = read_adc()
        if adc_value < min_val:  # Inverting comparison for brightest condition
            min_val = adc_value
        if adc_value > max_val:  # Inverting comparison for darkest condition
            max_val = adc_value
        utime.sleep(0.1)
        
        # Provide feedback to the user
        if i == samples // 2:
            print("Halfway through calibration...")
    
    print(f"Calibration complete. Min ADC value: {min_val}, Max ADC value: {max_val}")
    return min_val, max_val

# Main function
def main():
    min_val, max_val = calibrate_ldr()
    
    while True:
        adc_value = read_adc()
        light_level = map_adc_to_light_level(adc_value, min_val, max_val)
        print(f"ADC Value: {adc_value}, Light Level: {light_level:.2f}%")
        utime.sleep(1)

if __name__ == "__main__":
    main()
