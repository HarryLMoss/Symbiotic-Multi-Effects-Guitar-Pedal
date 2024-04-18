# Symbiotic Multi-Effects Guitar Pedal

This repository houses the Symbiotic Multi-Effects Guitar Pedal project, a comprehensive endeavor undertaken as part of my 3rd year University project for a BEng Electronic Engineering degree at the University of Manchester, UK. This project spanned from September 2021 to May 2022 and showcases a blend of hardware and software engineering skills, focusing on digital signal processing for audio effects.

## Overview
The Symbiotic Multi-Effects Guitar Pedal is an innovative solution for electric guitarists, offering a suite of effects including gain, distortion, echo, and reverb, all programmable via an easy-access user-friendly interface that is attached to the guitar body as opposed to the floor. Developed using C and Assembly language on a Blackfin BF706 processor, this project demonstrates my capability in embedded system design, digital signal processing, and low-level programming.

## Features
- **Gain Control:** Adjust the input signal's amplitude.
- **Distortion:** Offers both heavy and crunch distortion modes.
- **Echo:** Customizable echo length and decay.
- **Reverb:** Simulates different environments, such as halls and rooms.
- **Hardware Interaction:** GPIO for effect toggling and LED feedback.
- **Flash Memory Management:** For storing user presets.



## Getting Started

### Prerequisites
- ADSP-BF706 Development Board
- CrossCore Embedded Studio for development
- Basic understanding of digital signal processing and embedded systems

### Installation
1. Clone this repository to your local machine.
2. Open the project in CrossCore Embedded Studio.
3. Ensure the ADSP-BF706 board is connected to your PC.
4. Build the project and flash it to the board following the instructions on [this document](https://github.com/HarryLMoss/Symbiotic-Multi-Effects-Guitar-Pedal/blob/5e248a305c71987d566fe53f30ccaba5e6649da4/Drivers/flash%20programming.docx).

## Usage
1. Power on the ADSP-BF706 board.
2. Connect your electric guitar to the input jack and an amplifier to the output.
3. Use the GPIO pins to toggle between different effects.

## Contributing
Contributions to improve the Symbiotic Multi-Effects Guitar Pedal are welcome. Please follow these steps to contribute:

- Fork the repository.
- Create a new branch (git checkout -b feature-branch).
- Commit your changes (git commit -am 'Add some feature').
- Push to the branch (git push origin feature-branch).
- Create a new Pull Request.

## License
This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

## Acknowledgements
- Special thanks to my project supervisor, Dr. Patrick Gaydecki, for his guidance and support throughout the development process.
- Acknowledgement to the University of Manchester and the Department of Electrical & Electronic Engineering for providing the resources and environment conducive to this project's success.

## Additional Resources
- [Project Report](https://github.com/HarryLMoss/Symbiotic-Multi-Effects-Guitar-Pedal/blob/b745425271cbf792a98e93e68626e8cd2b41ad5a/Reports/FinalReport.docx)
- [Program Code](https://github.com/HarryLMoss/Symbiotic-Multi-Effects-Guitar-Pedal/blob/main/Program.c)
- [Demo Video](https://drive.google.com/file/d/1bZtaHuSL4E7lbnmdPmNMsFeQU9mE2X98/view?usp=sharing)

## Contact
For any inquiries or collaboration requests, please contact me at harrymoss33@gmail.com.

Other AI/Digital Signal Processing projects can be found on my [GitHub Profile](https://github.com/HarryLMoss)

---

For more information on how to use this project, refer to the various reports and product photos located within this repository.

© 2024 Harry Moss. All Rights Reserved.
