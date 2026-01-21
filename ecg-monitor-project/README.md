[![pipeline status](https://gitlab.oit.duke.edu/EmbeddedMedicalDevices/duke-bme554-ecg-temp-ble-lab/badges/main/pipeline.svg)](https://gitlab.oit.duke.edu/EmbeddedMedicalDevices/duke-bme554-ecg-temp-ble-lab/-/commits/main) 

# Duke BME 554: ECG, Temperature, and BLE Device Labs

This repository is associated with BME 554 and is solely for educational purposes.

Using the nRF52833 SDK, this project integrates fundamental embedded systems concepts including PWM, ADC, I2C communication, and Bluetooth to create a comprehensive ECG-like monitoring device. Various coding structures in C were employed, including threads, timers, ADC (analog-to-digital conversion), and PWM (pulse width modulation).

## Project Repository Overview/Navigation

* `application/src/main.c` - main application code
* `application/CMakePresets.json` - CMake presets file (build configuration)
* `application/CMakeLists.txt` - build system configuration file
* `application/prj.conf` - Zephyr configuration file
* `testing/final_technical_report.ipynb` - Final written report for the project
* `.gitlab-ci.yml` - GitLab CI configuration file
* `.gitignore` - ignore files that are not needed in the git repository
* `.west.yml` - Zephyr west configuration file
* `testing/technical_report.ipynb` - Jupyter notebook for the technical report

## Key Features

ECG Heart Rate Monitoring: The device processes live ECG signals ranging from 40–200 bpm, synchronizing an LED to blink at the corresponding heart rate frequency while calculating real-time BPM values. A single-buffer technique was implemented where ADC values continuously overwrite the buffer via callback function, enabling real-time signal processing without excessive overhead. Heart rate is determined by counting signal cycles within a 4-second window, triggered when the ECG signal crosses a 300mV threshold during the R wave ascent (the ECG signal was formed via a waveform generator and ranged from -500mV to 500mV). This threshold placement minimizes noise interference by avoiding both the signal extrema and baseline crossing regions where other ECG waves reside.


Battery Level Measurement: A PWM-based battery monitoring system was implemented using a linear mapping that converts 0–3V input into 0–100% duty cycle output​. The duty cycle corresponds directly to battery percentage, where 0% duty cycle indicates 0% battery and 100% duty cycle indicates 100% battery. Accuracy was validated through multi-method testing using a DC power supply connected to the AIN0 channel, with measurements verified via logging statements, Bluetooth outputs, and oscilloscope readings (see repository for verification report).


Temperature Sensing: An MCP9808 temperature sensor was integrated via I2C communication to provide on-demand temperature readings triggered by button press. The I2C protocol enables reliable digital communication between the nRF board and the temperature sensor.


BLE Data Transmission: Bluetooth functionality was implemented to wirelessly transmit battery voltage, temperature readings, and heart rate data to external devices, enabling real-time remote monitoring of all sensor outputs.

​​
The project demonstrates the integration of multiple embedded systems concepts, with each sub-function validated for accuracy through testing procedures including statistical analysis (mean, standard deviation, 95% confidence intervals) and comparison against different data measurement methods.

