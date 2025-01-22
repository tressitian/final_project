# final_project: Tree-hole Messenger

**Overview**

The Tree-hole Messenger is a device designed for long-distance relationships. It allows users to record voice messages that convey emotions without revealing the exact content. The device analyzes the tone and sentiment of the message and displays it through LED colors on a trunk-like wheel, enabling emotional connection even when words are not exchanged. Below is a general sketch highlighting its physical features:

**Sensor Device**

The sensor device consists of a microphone and a sentiment analysis module. The microphone records voice messages, and the sentiment analysis module powered bynChatGPT processes the tone and sentiment of the audio. The device categorizes the sentiment into predefined emotional states and translates it into a corresponding LED color. The system includes:

Microphone: [MAX9814 Electret Microphone Amplifier]

Processor: [ESP32-WROOM-32]

**Sketch**
1. three original ideas, and the third idea is my favorite. 
![5761736568834_ pic](https://github.com/user-attachments/assets/6514f84f-6bc6-4c68-81ba-a56b3ebb1fa4)

2. "sensor" device
![5841737530351_ pic](https://github.com/user-attachments/assets/ebda8192-0211-47f1-8dd9-ad948c635084)

3.  "display" device
![5851737530353_ pic](https://github.com/user-attachments/assets/2605e4e2-7b48-4801-8f70-5c2a0ccd6fb8)

**System Communication**

The Tree-hole Messenger employs a local or remote communication protocol, such as Bluetooth or Wi-Fi, to transmit data between the sensor device and the display device. The recorded audio is analyzed locally or on the cloud, and the resulting emotional state is sent to the display unit to update the LED colors.

![5861737531156_ pic](https://github.com/user-attachments/assets/40706815-5cb0-4f95-91d0-3710b51f4d2c)
