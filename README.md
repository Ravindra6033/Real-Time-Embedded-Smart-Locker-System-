Description: This project is a high-precision, cyber-physical security solution that leverages deterministic weight sensing to manage locker rentals. Unlike standard electronic lockers that rely solely on PIN entry, this system uses a 24-bit ADC pipeline to verify the physical presence of items in real-time, preventing "ghost" bookings and ensuring a robust audit trail.


Components Used: Raspberry Pi 5, HX711 Load cell with ADC, Solenoid Lock, 3d Printed platform, external locker and hinges


Hardware Setup: HX711 --> Raspberry PI

VCC 3.3V --> Pin 1

GND --> Pin 6

DT --> Pin 29

SCK --> Pin 23



Load Cell --> HX711

Red wire --> E+

Black Wire --> E-

White --> A-

Green --> A+



Repository Contents
1. Main File - The initial tested code
2. code.cpp - the code modified while calibrating the sensors while testing the hardware
3. smart locker.cpp - A refined version addressing real-time constraints. It includes optimized timing loops to resolve synchronization issues between the load cell data rate and the main execution thread, ensuring no data packets are dropped during polling.
4. features.cpp - The final, feature-complete implementation. This version integrates:
Web Dashboard: Real-time observation of locker occupancy and pricing.
Event Logging: Deterministic system logging that records every lock/unlock event and item detection timestamp.
Enhanced UI: Responsive HTML/JS interface for user interactions and PIN management.
5. rt features update.cpp - the final code with all the modifications involved and all the measures taken to ensure smooth function of the system and no hiccups of real time data observation
6. CmakeLists.cpp - all the external libraries directories

 Other Files in the Repository
 1. Real Time Embedded Smart Locker System - the Idea presentation PPT
 2. Intro Video.mp4 - our first introduction video uploaded on social media handles 


 Social Media:
 Instagram - https://www.instagram.com/realtimeembeddedproject5/ 
