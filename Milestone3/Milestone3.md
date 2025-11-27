# Smart Robot Project

## Team Member  
- **Cunningham Li**

## Adafruit IO Dashboard  
🔗 **Dashboard:** https://io.adafruit.com/cunninghamli/dashboards/smartpath  
<img width="1919" height="829" alt="image" src="https://github.com/user-attachments/assets/be3c223d-e9a7-4a30-99bf-deca459d2833" />



_(Private for security reasons)_

## Cloud Storage (Daily Log Uploads)  
🔗 **Google Drive Folder:** https://drive.google.com/drive/my-drive  
<img width="1919" height="993" alt="image" src="https://github.com/user-attachments/assets/6166c1c5-3f7a-4e35-8032-ce7d156317b3" />

_(Contains automatically uploaded daily logs)_

## Reflection  
Everything in the system is now fully functional. The robot successfully publishes live sensor data to Adafruit IO, the Flask dashboard reads it in real time, and all control pages work including LED, buzzer, manual motor commands, and the three driving algorithms. The cloud database (Neon) also stores every sensor reading without issues, and the history page correctly plots past data by date. The hardest part of the whole project was integrating all components together, because each piece: MQTT, Flask, sensors, hardware drivers, and database, behaves differently and must stay perfectly synchronized. Debugging the Pi’s connectivity and feed naming issues also took time since one broken feed stopped entire pages from updating.

The major improvement I would make in the future is implementing a more advanced offline-sync system with conflict detection, so local readings merge intelligently with the cloud when the Pi reconnects. I would also add pagination to the history dashboard because the Neon table will grow large over time. Another improvement would be to containerize the entire project using Docker, which would make deployment onto another Raspberry Pi much simpler. Finally, creating more polished UI pages and adding camera streaming would make the dashboard feel more complete and closer to a production-ready IoT system.

## Demo Video  
📺 **Link:** https://1drv.ms/v/c/5ec1dadb80ca3cd5/EUK2QO21kttFvv5Y7yX93boBUIa22mB7-qtqGEQtCJwZ6g?e=5hCica

