# ⚙️ Fuzzy-DC-Motor-Control: Error Zeroing Optimization

This project features a simulation of a Mamdani-type Fuzzy Logic Controller (FLC) designed for DC motor speed control. The main goal is to ensure the motor speed reaches the reference target of **100 rad/s**, thereby **eliminating the steady-state error**.

## 🎯 Key Optimizations

To solve the persistent error observed in the original code, two primary changes were implemented:

1.  **Sensitivity Boost:** The membership functions for Error (`e`) and Change in Error (`de`) were narrowed (especially the `Z` sets) to ensure a high-gain response to small errors.
2.  **Output Gain (Gu):** An output gain of **Gu = 2.0** was added within the `simulate` function. This aggressively drives the motor with maximum voltage to quickly reach the target.
3.  **Simulation Time:** The simulation duration was extended to **T=15.0 seconds** to fully observe the error settling to zero.

## 📈 Result: Error Successfully Zeroed

Following the optimizations, the motor speed reached the target in approximately 6 seconds, and the **error was permanently eliminated** (zeroed).
<p align="center">
  
<img  alt="Figure_4" src="https://github.com/user-attachments/assets/209c477c-daa2-4f32-81d2-71d9ab8609a5" width="450" />
<img  alt="Figure_3" src="https://github.com/user-attachments/assets/eafe3fdb-7a4e-45f7-8ab8-95ed0d613fc9" width="450"/>
<img  alt="Figure_2" src="https://github.com/user-attachments/assets/d4a11f2c-ae93-4b44-8dbb-04baed2735be" width="450"/>
<img  alt="Figure_1" src="https://github.com/user-attachments/assets/e084f59e-43bf-41f3-8674-797f15222f39" width="450"/>
</p>
