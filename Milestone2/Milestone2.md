# Smart Robot Project

## Team Member  
- **Cunningham Li**

## Adafruit IO Dashboard  
🔗 **Dashboard:** https://io.adafruit.com/cunninghamli/dashboards/smartpath  
_(Private for security reasons)_

## Cloud Storage (Daily Log Uploads)  
🔗 **Google Drive Folder:** https://drive.google.com/drive/my-drive  
_(Contains automatically uploaded daily logs)_

## Project Overview  
This smart robot project features a Raspberry-Pi-powered car controlled remotely using MQTT and Adafruit IO. The system supports manual driving, line-assist mode, simulated sensor feedback, and automatic file uploads using rclone. Telemetry is logged locally and synced to the cloud.

## Reflection  
Throughout this project, I successfully built a remotely controlled smart car system with MQTT, simulated sensors, and a working Adafruit IO interface. The robot responded to dashboard inputs, logged telemetry locally, and could operate in both manual control and line-assist mode. The part that worked best was the remote dashboard commands and real-time feedback, since I could clearly see the robot reacting to my inputs.

The hardest part was debugging sensor behavior and making sure the robot reacted safely when readings were stale or below the obstacle threshold. Getting the line and distance logic consistent, and ensuring the robot reversed correctly instead of moving forward, required a lot of testing. Another challenging area was setting up rclone and file logging automation, because it involved Linux tools and cloud configuration I had never used before.

If I had more time, I would integrate real hardware sensors once they work consistently instead of relying on simulation, and improve the dashboard UI for smoother control. I would also add a “simulation mode” toggle and possibly live camera streaming. Overall, I learned a lot about IoT communication, safety logic, and debugging embedded systems.

## Demo Video  
📺 **Link:** https://slcqc-my.sharepoint.com/:v:/g/personal/2231358_champlaincollege_qc_ca/EdroHz7UQ61Or6kDf5RQf34BK0nIdUpyyM62thvEYGQWRA?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=UaA1zk 
