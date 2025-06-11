# 1rst Test

<div align="justify">

## General Introduction
As part of the TekBot Robotics Challenge 2025 (TRC2K25), this project aims to design and implement a motion detection and analysis system based on an inertial sensor combining a gyroscope and an accelerometer. These sensors play a fundamental role in modern robotics, virtual reality, navigation, and many applications requiring precise measurement of an object's orientation and movement dynamics.

The project focuses on leveraging the capabilities of a sensor such as the MPU6050, which integrates both a gyroscope and an accelerometer in a single component, communicating via the I2C protocol. Using an Arduino microcontroller, we will read raw data from the sensor, analyze the direction and speed of hand movement, and display this information in real-time on an LCD screen.

Simultaneously, a suitable autonomous power supply will be designed to provide energy for the device. This project thus allows us to apply a variety of skills in electronics and embedded programming while addressing a practical challenge of motion detection for robotics.

## Objectives

## Power Supply Sizing

### Choice of Transformer 

To ensure galvanic isolation between our power supply and the 220V mains, we have chosen a **220V/15V - 30VA** step-down transformer.  

The apparent power of the secondary is given by the formula:  

<center>S<sub>2</sub> = U<sub>2</sub> x I<sub>2</sub></center><br />

By substituting the values:  

<center>I<sub>2</sub> = <sup>S<sub>2</sub></sup> / U<sub>2</sub> = <sup>30</sup> / 15 = 2A</center>
<br />

This current is sufficient for our application.  

The primary of the transformer will be connected to the **220V** mains, while the secondary will provide a voltage of approximately **15V**, which will be used to power our circuit.
<br />

### Choice of Rectifier

Since we are operating at a frequency of **50Hz**, standard diodes can be used for rectification.  
We opted for **full-wave rectification** because it offers several advantages over **half-wave rectification**, particularly in the sizing of the filtering capacitor, which we will discuss later.  

Additionally, the diodes must withstand:  
- A **direct average current** of **2A**  
- A **reverse voltage** (V<sub>rr</sub>) of **21.21V**  

Thus, we can select **3A/40V** diodes for our application.  

For **recycling purposes**, we used a **molded bridge rectifier** rated at **3A/400V**, salvaged from an old PC power supply.  
Its high reverse voltage tolerance makes it a perfect fit for our needs.  
<br />

### Choice of Filter Capacitor

The capacitor value is determined using the following formula:

<center>C = <sup>I<sub>e</sub></sup> / (F . ∆V<sub>e</sub>)</center>
<br />

- **∆Ve**: The voltage ripple at the regulator input. We set **∆Ve = 3.5V**.  
- **F**: The frequency of the voltage at the bridge rectifier output, **F = 100 Hz** (due to full-wave rectification: 50 Hz × 2).  
- **Ie**: The regulator input current, defined as **Ie = Is + Ireg**, where **Ireg** is the bias current consumed by the regulator itself under load.  

In the worst-case scenario, we assume **Is = 2A** and **Ireg = 10mA**, resulting in **Ie = 2.01A**.  

Thus, when the power supply load reaches **2A**, the voltage ripple **∆Ve** across the capacitor must be **3.5V**.  

After calculation, we find **C = 6483.9 µF**. A standardized value close to this is **6800 µF**.  
In practice, the capacitance **C** can be higher, but it should not exceed **10,000 µF**, as excessive capacitance can cause issues with rectifier diodes (a very large capacitor behaves like a short circuit when fully discharged).  

To achieve this value, we can use **two 4700 µF capacitors in parallel**, considering component tolerances.  

The working voltage of these capacitors must be higher than the maximum voltage at the bridge rectifier output.  
This maximum voltage is **15√2 = 21.21V**, so we select a working voltage of **25V**.  

Finally, we choose **one 4700µF/25V capacitor and one 3300µF/25V capacitor**, connected in parallel, resulting in an equivalent capacitance of **8000µF/25V**.  
<br />

### Choice of Regulation Type

#### LM7805 regulator limit

---> **Current Constraint**
- The **7804** regulator is limited to a maximum of **1.5A**.  
- A quick solution is to use a **ballast transistor** alongside the regulator to provide the required current.  
- However, this introduces a **thermal dissipation constraint**, as significant power needs to be dissipated.  

---> **Power Dissipation**
- The regulator's input voltage ranges from **10.5V to 13V**.  
- For a current of **1A**, the power dissipation is approximately **10.5W**.  
- This necessitates the **design of an appropriate heatsink** for the power supply. 
<br /> 

#### Choice of buck converter LM2596-5

Optimal Regulator Selection

---> **Advantages of the Chosen Regulator**
- **Nominal current of 3A**, which is well-suited for our application.  
- With an **external current limiting circuit**, we can restrict the current to **2A**, as required.  
- As an **all-in-one buck converter**, it provides **excellent line and load regulation**.  
- **Power dissipation is relatively low**, eliminating the need for a **heatsink** for the module.  

---> **Regulator Choice**

Given these constraints, **linear regulation** would not be advantageous.  
The **LM2596-5** is an **optimal solution** for our application.  

---> **Implementation**
For the design, we selected a **module integrating**:  
- The **LM2596-5 regulator**,  
- **Decoupling and filtering capacitors**,  
- **Smoothing inductor**,  
- **Freewheeling diode**.  
<br />

### Selection of the Protection Resistor for the Indicator LED

---> Selection of the Protection Resistor for the Green LED

We have chosen a **green LED** with: 

- **Forward voltage**: V<sub>LED</sub> = 2.1V 
- **Average current**: I<sub>D</sub> = 20mA  

With a **5V output voltage**, the resistor value is calculated as follows:

<center>R = <sup>(V<sub>supply</sub> - V<sub>LED</sub>)</sup> / I<sub>D</sub> = <sup>(5V - 2.1V)</sup> / 20mA = 145Ω</center>
<br />

A standard value in the **E24 series** is **150Ω**.  

---> **Power Dissipation**

The power dissipated by the resistor is:

<center>P<sub>D</sub> = R x I<sub>D</sub><sup>2</sup> = 145 x (20mA)<sup>2</sup> = 58mW</center>
<br />

Thus, we select a **150Ω, 1/4W** resistor (250mW), ensuring sufficient safety margin.
<br />

### Current Limiting Cell Design

-------------------------------------------------image--------------------------------------------

This circuit is based on **two main components**:  
- **Transistor**: **2N2222**  
- **Optocoupler**: **PC817**  

---> **Operation**  

- When **transistor Q1** is **OFF**, the **optocoupler diode** is **OFF**, keeping the **phototransistor OFF**.  
- This maintains the **ON/OFF pin** of the regulator at a **low state** (**V<sub>ON/OFF</sub> < 1.3V**), activating the regulator and allowing **output voltage presence**.  

The **R4 resistor** acts as a **shunt resistor** for current measurement.  
- The **voltage across R4** is proportional to the current flowing through it.  
- **R4 is chosen** to produce a **0.7V drop** when the current reaches **2A**.  

----> **Case 1: Current < 2A**  
- The voltage drop across **R4** is **less than 0.7V**, keeping **Q1 OFF** and the regulator **ON**.  

----> **Case 2: Current ≥ 2A**  
- The voltage drop exceeds **0.7V**, **activating Q1**.  
- The **optocoupler diode** turns **ON**, making the **phototransistor conductive**.  
- **R1 and R2** form a **voltage divider**, polarizing **V<sub>ON/OFF</sub>** above **1.3V**, **disabling the regulator**.  

Thus, **current limitation** is achieved.  

---> **Resistor Calculations**  

----> **R4 Value**  

<center>R<sub>4</sub> = <sup>0.7</sup> / 2A = 0.35Ω = 350mΩ</center>
<br />

----> **R3 Value**  
The **R3 resistor** protects the **optocoupler diode**, limiting its current to **20mA**.  


<center>R<sub>3</sub> = <sup>(V<sub>in</sub> - V<sub>threshold</sub> - V<sub>CEsat</sub>)</sup> / I<sub>F</sub></center>
<br />

With:  
- **V<sub>in</sub> = 12.5V**  
- **V<sub>threshold</sub> = 1.2V**  
- **V<sub>CEsat</sub> = 0.2V**  
- **I<sub>F</sub> = 20mA**  

<center>R<sub>3</sub> = <sup>(12.5V - 1.2V - 0.2V)</sup> / 20mA = 555Ω</center>
<br />

Thus, we select a **600Ω, 1/4W** resistor (250mW), ensuring sufficient safety margin.

<br />
<br />

### Conclusion

## Implementation of the Navigation System

</div>