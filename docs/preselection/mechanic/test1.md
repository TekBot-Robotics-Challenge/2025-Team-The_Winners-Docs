# 1rst Test

### Assembly test

This section of the test consists of assembling different parts to obtain a mechanical gripper. After downloading the ZIP file, we proceed with assembling the provided parts while ensuring that the necessary constraints are applied.

---

### Clarification of Concepts

Assembly in **SolidWorks** is an essential step in 3D modeling, allowing you to combine multiple individual parts (called components) to form a complete and functional mechanical system. With this feature, you can:

- Simulate the real behavior of a mechanism,  
- Check movements and interferences,  
- Analyze physical properties like mass and center of gravity.

---

### Constraints and Features Used for the Gripper

#### 🔩 "Coaxial" Constraint

The coaxial constraint aligns two cylindrical or circular axes (holes, shafts, cylinders) so that they share the same central axis.

**Example:** aligning a shaft with its corresponding bore.

---

#### 📏 "Coincident" Constraint

The coincident constraint forces two flat or linear faces to touch — that is, to be **coplanar** or **merged**.

**Purpose:** to attach one part against another.

---

#### 🔁 "Symmetric Components" Feature

Part of the **“Linear Component Pattern”** section.  
It allows you to create a **symmetrical copy** of a parent component with respect to a **reference plane**.

---

### Assembling the Parts

#### Steps:

1. Open **SolidWorks**  
2. Open the `ASSEMBLAGE PINCE` file  
3. Complete the missing parts

#### Parts to Insert:

- Connecting rods  
- Jaw holder  
- Left jaw  
- Right jaw  
- Connecting rod pin  
- Jaw holder pin  
- CHC screw M5×16  
- CHC screw M5×25  
- Circlips  

#### Assembly Procedure:

1. **Insert the first connecting rod**:  
   - Use **coaxial** and **coincident** constraints  
   - Position it **above the actuator rod end**.

2. **Insert the second connecting rod**:  
   - Placed **underneath the first**, with the actuator rod end between them.  
   - Add a **coincident constraint** between the two connecting rod faces for perfect alignment.

3. Use the **"Symmetric Components"** command (with the top plane) to obtain 4 connecting rods.

4. **Insert the "jaw holder" part** using **coaxial** and **coincident** constraints, then mirror it as before.

5. **Insert the jaws (left and right)** and the fastening components:  
   - CHC screw M5×16  
   - CHC screw M5×25  
   - Connecting rod pin  
   - Jaw holder pin  
   - Circlips

> 🔧 Result: A complete **mechanical gripper**.

---

### Additional Tasks

#### 📍 Center of Gravity at the **Minimum Position** (gripper open)

- Fix the actuator rod end at the **minimum** position.

---

#### 📍 Center of Gravity at the **Maximum Position** (gripper closed)

- Fix the actuator rod end at the **maximum** position.
