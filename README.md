# [Major revision in progress] Quadruple-Tank Setup <br><sub> ⚗️ Reproducible Low-cost Flexible Quadruple-Tank Process Experimental Setup for Control Educators, Practitioners, and Researchers</sub> 

[![GitHub release](https://img.shields.io/github/release/Naereen/StrapDown.js.svg)](https://github.com/decenter2021/SAFFRON/releases)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/decenter2021/quadruple-tank-setup/blob/readme/LICENSE)
[![DOI:[not_published_yet]](https://zenodo.org/badge/DOI/not_published_yet.svg)](https://doi.org/not_published_yet.svg)

<img src="https://user-images.githubusercontent.com/40807922/163567003-0c99b331-ad43-42da-af84-5637b5629d2c.png" width="100%" /> |   <img src="https://user-images.githubusercontent.com/40807922/163566465-bf311f1e-8831-4145-b541-65d9046de89c.png" width="100%" /> 
:-------------------------:|:-------------------------:
CAD model             |  Physical implementation

***
## 🎯 Features
- User-friendly dedicated **MATLAB/Simulink** interface with a personal computer<br>
- Seamless shift between a **numeric simulation** and the **interface** with the real experimental plant<br>
- **Inexpensive** materials and **fast assembly** <br>
- **Open-source** <br>
  - CAD models
  - Technical drawings
  - Wiring schematics
  - PCB design
  - MATLAB/Simulink interface
  - Assembly tutorials
- Several application **examples**
- *[Specify application eamples here]*
- *[Flexibility]*

***
## 🚀 Index

- [Description](#-description)
- [Authors](#-authors)
- [Contact](#-contact)
- [Manual](#-manual)
- [Examples](#%EF%B8%8F-examples)
- [Parts list](#%EF%B8%8F-parts-list)
- [Contributing](#-contributing)
- [Lincense](#-license)
- [References](#-references)

***

## 💡 Description


The major hurdle in accessing laboratory experimentation is the <b>cost of acquiring experimental scientific equipment</b>, which is unbearable for many institutions. This repository provides the community of control <b>educators, practitioners, and researchers</b> with an <b>open-source low-cost</b> experimental setup and dedicated interface, which is <b>flexible</b> and very <b>easily reproducible</b>. 


<p align="justify">
The Quadruple-Tank Setup is thoroughly described in 
</p>
<p align="justify">
<a href="">Pedroso, L., Batista, P. (2022) Reproducible Low-cost Flexible Quadruple-Tank Process Experimental Setup for Control Educators, Practitioners, and Researchers [not published yet]</a>
</p>

If you use this repository, reference the publication above.



The community is encouraged to [contribute](#-contributing) with application examples and suggest improvements.
***

## ✍🏼 Authors 
Leonardo Pedroso<sup>1</sup> <a href="https://scholar.google.com/citations?user=W7_Gq-0AAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0002-1508-496X"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a> <a href="https://github.com/leonardopedroso"><img src="https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>
Pedro Batista<sup>1</sup> <a href="https://scholar.google.com/citations?user=6eon48IAAAAJ"><img src="https://cdn.icon-icons.com/icons2/2108/PNG/512/google_scholar_icon_130918.png" style="width:1em;margin-right:.5em;"></a> <a href="https://orcid.org/0000-0001-6079-0436"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a><br>

<sub>*<sup>1</sup>Institute for Systems and Robotics, Instituto Superior Técnico, Universidade de Lisboa, Portugal<br>*</sub>

***

## 📞 Contact
This repository is currently maintained by Leonardo Pedroso (<a href="mailto:leonardo.pedroso@tecnico.ulisboa.pt">leonardo.pedroso@tecnico.ulisboa.pt</a>).

***

## 📚 Manual

The **simulink block** for interfacing with the quadruple-tank process experimental network is shown below. The **inputs and outputs** are thoroughly described in <a href="#-references">(Pedroso and Batista, 2022)</a>. 

<p align="center">
<img src="https://user-images.githubusercontent.com/40807922/178805687-0e0324f4-316a-4057-89fc-b665e4af53d4.png" width="25%" /> 
 </p>

To create a **new model** to control the quadruple-tank experimental setup, follow these steps:
- Copy and paste the interface block in a new Simulink model;
- Copy and paste, in the directory of the new Simulink model, the file: 
  - `quadrupleLoadParameters.m`, which is executed on the initialization callback of the interface block, that loads the physical parameters of the setup, defines the sampling period, and defines the covariance matrices of the process and sensor noise for numerical simulation;
  - `quadrupleSimObj.m`, a MATLAB System Object that numerically simulates the quadruple-tank process in discrete-time;
- Create a directory `identification` to contain the `.mat` data files generated in the [identification procedure](#-identification);
- Generate and connect to the interface block a clock signal `CLK`, whose period equals the sampling period defined in `quadrupleLoadParameters.m`;
- Setup the Simulink solver to:
  - Fixed-step;
  - Discrete; and
  - Set the "Fixed-step size" parameter to half the period of the `CLK` signal;
- Set the Simulink simulation mode to "Accelerator";
- Enable the display of the "Sorted Execution Order" of Simulink to ensure the first step of the feedback loop is the water level measurement and the last is the update of the input to the pumps. Most of the time this is ensured, due to the default priorities of the inner blocks of the interface block, otherwise set the priorities of the feedback loop blocks appropriately.


***

## 🧐 Identification

***

## ⛳️ Examples

<p align="justify">
The source files of the applications described in <a href="#-references">(Pedroso and Batista, 2022)</a> are available in this repository. Their execution is shown for a prototype in the videos linked below. Access the full playlist <a href="https://www.youtube.com/playlist?list=PLj6JXRV4qcgboMBOonhQY3f7nb1WQ0N1v">here</a>.
</p>

Description             |  Video
:-------------------------:|:-------------------------:
3D Printed Valve Assembly Tutorial | <a href="https://www.youtube.com/watch?v=pIbw3Wvdszw"><img src="https://user-images.githubusercontent.com/40807922/163651252-2209618e-cd7d-4c94-bd00-5b37f622e6db.png" width="80%" /></a>
PI Control Example | <a href="https://www.youtube.com/watch?v=r2xKSpMdZIU"><img src="https://user-images.githubusercontent.com/40807922/163651390-56039e46-3c19-4354-8ae3-b6aff416eb6e.png" width="80%" /></a>
Frequency Response Example | <a href="https://www.youtube.com/watch?v=jRHPns1iMps"><img src="https://user-images.githubusercontent.com/40807922/163650886-a79de64e-970b-4af1-92a2-a5f4cea1f8fb.png" width="80%" /></a>
LQR Control Example | <a href="https://www.youtube.com/watch?v=Tk9IJJCzURs"><img src="https://user-images.githubusercontent.com/40807922/163651541-476bd066-3dea-4e89-b7f1-8f1a0621f1a3.png" width="80%" /></a>
Decentralized LQR Control Example | <a href="https://www.youtube.com/watch?v=NA25sSz-3jE"><img src="https://user-images.githubusercontent.com/40807922/163650886-a79de64e-970b-4af1-92a2-a5f4cea1f8fb.png" width="80%" /></a>

***

## ⚙️ Parts list

The full parts list is availble in **spreadsheet** format in [bill-of-materials/bill-of-materials.xlsx](https://github.com/decenter2021/quadruple-tank-setup/tree/master/bill-of-materials). 

Below the parts are listed among categories, with correponding cost as of **2021**. The total cost is **under 650€**.

### 💎 Acrylic

Part | Technical drawing  | Quantity | Total cost
:---|:---|:---:|:---:
Reservoir | [bottom_tank.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/bottom_tank.pdf) | 1 | 39€
Cylindrical tank with base | [tank_small.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/tank_small.pdf) / [tank_large.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/tank_large.pdf) | 4 | 133€
Slab \#1 | [slab_1.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/slab_1.pdf) | 1 | 8€
Slab \#2 | [slab_2.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/slab_2.pdf) | 1 | 8€
Slab \#3 | [slab_3.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/slab_3.pdf) | 1 | 8€
Slab \#4 | [slab_4.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/slab_4.pdf) | 1 | 8€
Slab \#5 | [slab_5.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/slab_5.pdf) | 1 | 8€
Cylindrical tube for sensor support | [sensor_tube.pdf](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/technical-drawings/sensor_tube.pdf) | 4 | 11€

### 🔩 Structural

Part | Quantity | Total cost
:---|:---:|:---:
Zinc-plated threaded steel rod M8 x 1000mm | 5 | 5€
ISO 4034 - M8 Hexagon Nut | 60 | 2€
ISO 7093 - 8 Washer | 45 | 9€
ISO 7091 - 8 Washer | 15 | 2€
ISO 4015 - M4 x 20mm Hexagon Head bolt  | 32 | 3€
ISO 4035 - M4 Hexagon thin nuts chamfered | 32 | 2€
ISO 7092 - 4 Washer  | 64 | 3€
M3 x 15mm Hex Spacer Female-Female | 12 | 3€	
ISO 7045 - M3 x 14mm bolt | 12 | 2€
ISO 7045 - M3 x 6mm bolt | 12 | 2€
Transparent flexible tubing 8mm x 11mm x 5000mm | 1 | 6€
Transparent flexible tubing 12mm x 16mm x 5000mm | 1 | 10€
Suction cup | 2 | 2€
Rubber Washer 8mm x 14mm x 2mm |  2 | 0.5€
Rubber Washer 9mm x 14mm x 2mm | 4 | 1€
Rubber Washer 8mm x 12mm x 2mm | 4 | 1€
Teflon tape 12 m | 2 | 1€

### 🖨️ 3D printed 

Part | Solid Edge part file | Quantity | Total cost
:---|:---|:---:|:---:
Rod support | [rod_base.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 5 | 1.5€
Nut (Three-way valve) | [nut_diverting.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 2 | 0.5€ 
Pin (Three-way valve) | [regulator_pin_diverting.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 2 | 0.5€
Body (Three-way valve) | [body_diverting.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 2 | 1€
Nut (Upper/lower tank outlet valve) | [nutStraight.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 4 | 0.5€
Pin (Upper/lower tank outlet valve) | [regulator_pin3_straight.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 4 | 0.5€
Body (Upper tank outlet valve) | [body3_straight.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 2 | 0.5€
Body (Lower tank outlet valve) | [body3_straight_bottom.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 2 | 0.5€ 
Upper tank cap | [cap_tank_small.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 2 | 2€
Lower tank cap | [cap_tank_large.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 2 | 2€	
Sensor mount | [sensor_support.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 8 | 2€
Flexible tube spacer D8 | [tube_support.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 4 | 0.5€
Flexible tube spacer D12 | [tube_support_large.par](https://github.com/decenter2021/quadruple-tank-setup/tree/master/cad-quadruple-tank/3d-printing-parts) | 1 | 0.5€


### 🔌 Connectors

Part | Quantity | Total cost
:---|:---:|:---:
USB 2.0 A - mini USB B cable | 1 | 2€
Multifilar 0.5mm$^2$ black wire - 5m | 1 | 7€
Multifilar 0.5mm$^2$ red wire - 5m | 1 | 7€
Multifilar 0.14mm$^2$ black wire - 5m | 1 | 3€
Multifilar 0.14mm$^2$ red wire - 5m | 1 | 3€
Multifilar 0.14mm$^2$ white wire - 5m | 1 | 3€
5.5/2.1mm male DC plug | 1 | 0.5€
5.5/2.1/14mm female DC plug | 4 | 1€
NS25-G3 NINIGI plug | 2 | 0.5€
NS25-G4 NINIGI plug | 8 | 1€
NS25-G6 NINIGI plug | 3 | 0.5€
NS25-T NINIGI contact | 56 | 3€
NS25-W3P NINIGI socket | 2 | 0.5€
NS25-W4P NINIGI socket | 4 | 1€
NS25-W6P NINIGI socket | 3 | 1€
NSR-06 NINIGI plug | 2 | 0.5€
NDR-T NINIGI contact | 12 | 1.5€
10-pin 2.54mm single row female pin header | 2 | 0.5€
15-pin 2.54mm single row female pin header | 2 | 1€

### ⚡ Electronics
Part | Quantity | Total cost
:---|:---:|:---:
Arduino Nano | 1 | 7€
Continuous Fluid Level Sensor PN-12110215TC-12 | 4 | 136€
VMA421 water pump | 4 | 72€
100uF 25V electrolytic capacitor | 4 | 0.5€
L298N Dual H-Bridge Driver | 2 | 9€
ADS 1115 ADC | 2 | 20€
14VDC 2.5A 35W power supply | 1 | 21€
Bi-stable emergency button | 1 | 12€
L7805ACP voltage regulator | 1 | 3€
Custom PCB with connectors | 1 | 35€

***


## ✨ Contributing

The community is encouraged to contribute with 
- Suggestions
- Application examples

To contribute

- Open an issue ([tutorial on how to create an issue](https://docs.github.com/en/issues/tracking-your-work-with-issues/creating-an-issue))
- Make a pull request ([tutorial on how to contribute to GitHub projects](https://docs.github.com/en/get-started/quickstart/contributing-to-projects))
- Or, if you are not familiar with GitHub, [contact the authors](#-contact) 

***

## 📄 License
[MIT License](https://github.com/decenter2021/quadruple-tank-setup/blob/readme/LICENSE)

***

## 💥 References 
<p align="justify">

<a href="">Pedroso, L., Batista, P. (2022) Reproducible Low-cost Flexible Quadruple-Tank Process Experimental Setup for Control Educators, Practitioners, and Researchers [not published yet]</a>

</p>
