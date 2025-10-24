# COMP3000 Project Title - Secure Wearable Authentication System for Sports Telemetry 

# Supervisor 
Ji-Jian Chin


# Project Vision
As of 2025, around 454.69 million people use smartwatches worldwide. Smartwatches and wearable devices are now essential in in everyday use and sports with 92% of users relying on their devices for health and fitness tracking. This authentication system is designed for sports professionals, athletes, and everyday smartwatch users who rely on wearable devices to monitor health and performance metrics such as heart rate, temperature, and movement. Telemetry and biometric data are vulnerable to interception, spoofing, and misuse due to a lack of robust cybersecurity protection in current wearable technologies.

The aim of this project is to design and implement a secure authentication framework for a wearable telemetry device that ensures data is only transmitted when the device is worn by the legitimate user and is cryptographically protected against attacks. 

To implement this framework, a prototype will be built using a low-cost microcontroller such as ESP32, and sensors including a heart-rate monitor(PPG), accelerator, temperature sensor and skin-contact detector.

The system will work by:

•	Ensuring that the legitimate user is wearing the device by authenticating them using physiological and motion-based signals.

•	Encrypting telemetry data using AES-GCM symmetric encryption.

•	Using RSA digital signatures to verify device authenticity.

•	Automatically locking/stopping transmission if it is removed from the users wrist.

•	Adapting to short-term physiological changes such as illness or stress by using multiple sensors, adaptive thresholds and fallback mechanisms. This ensures the system doesn’t falsely reject the user when their biometric readings will naturally vary.

This project addresses cybersecurity issues such as insecure data transmission, device spoofing and provides robust authentication by distinguishing genuine physiological variability from unauthorized device use. 


