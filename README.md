# ROS-Web Integration Challenge

A comprehensive web-based robot control system that integrates ROS (Robot Operating System) with modern web technologies to provide remote control capabilities for TurtleBot3 Waffle simulation.

## 🎯 Project Overview

This project demonstrates a real-time web interface for controlling a TurtleBot3 robot, featuring live camera streaming, teleoperation controls, system monitoring, and comprehensive logging capabilities. The system is designed to meet industrial IoT requirements with scalable microservices architecture.

## 🏗️ Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Web Frontend  │◄──►│  Express.js API │◄──►│  MySQL Database │
│   (Browser)     │    │   (Node.js)     │    │   (Logging)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                        │
         ▼                        ▼
┌─────────────────┐    ┌─────────────────┐
│  ROS Rosbridge  │    │  Web Video      │
│  (WebSocket)    │    │  Server (HTTP)  │
└─────────────────┘    └─────────────────┘
         │                        │
         ▼                        ▼
┌─────────────────────────────────────────┐
│         TurtleBot3 ROS System           │
│    (/cmd_vel, /odom, /camera/*)         │
└─────────────────────────────────────────┘
```

## ✨ Features

### Core Functionality
- **Real-time Robot Control**: Web-based teleoperation with multiple input methods
- **Live Camera Streaming**: MJPEG video feed from robot's camera
- **System Monitoring**: Real-time health checks and status monitoring
- **Comprehensive Logging**: MySQL-based logging system with 1-hour retention
- **Robot Management**: Start/Stop robot containers remotely

### Control Methods
- **Directional Buttons**: Click-based movement controls (▲▼◀▶)
- **Keyboard Control**: WASD and arrow keys with Shift for speed boost
- **Virtual Joystick**: Touch-friendly joystick interface using NippleJS
- **Touch Support**: Mobile-responsive touch controls

### Advanced Features
- **Snapshot Capture**: Save camera frames with server-side storage
- **Auto-Reconnection**: Robust WebSocket connection management
- **Real-time Telemetry**: Live odometry data display
- **Responsive Design**: Mobile-first responsive web interface

## 🛠️ Technology Stack

### Frontend
- **HTML5/CSS3**: Modern responsive design
- **Vanilla JavaScript ES6+**: No framework dependencies
- **WebSocket API**: Real-time communication
- **Canvas API**: Image processing for snapshots
- **NippleJS**: Virtual joystick library
- **RosLib.js**: ROS WebSocket bridge client

### Backend
- **Node.js**: JavaScript runtime
- **Express.js**: Web application framework
- **MySQL2**: Database driver with connection pooling
- **Dockerode**: Docker API client for container management

### ROS Integration
- **ROS Noetic**: Robot Operating System
- **Rosbridge Suite**: WebSocket bridge for web integration
- **Web Video Server**: HTTP-based video streaming
- **TurtleBot3 Packages**: Robot simulation and control

### DevOps
- **Docker & Docker Compose**: Containerization and orchestration
- **Multi-container Setup**: Service isolation and networking
- **Volume Management**: Persistent data and asset storage

## 📋 Requirements

- **Docker**: Version 20.0+ with Docker Compose
- **Hardware**: Minimum 4GB RAM, 2 CPU cores
- **Browser**: Chrome/Firefox/Safari with WebSocket support
- **Network**: Ports 80, 8080, 9090 available

## 🚀 Quick Start

### 1. Clone Repository
```bash
git clone https://github.com/dietrichtarigan/itbdelabo-ros-web-integration-challenge.git
cd itbdelabo-ros-web-integration-challenge
```

### 2. Start Services
```bash
docker compose up -d --build
```

### 3. Verify Services
```bash
# Check all containers are running
docker compose ps

# Expected output:
# tbt3_robot   running   0.0.0.0:8080->8080/tcp, 0.0.0.0:9090->9090/tcp
# tbt3_web     running   0.0.0.0:80->3000/tcp  
# tbt3_db      running   3306/tcp
```

### 4. Access Web Interface
Open your browser and navigate to:
```
http://localhost
```

## 🔧 Service Configuration

### Port Mapping
- **Port 80**: Web Interface (Frontend + API)
- **Port 8080**: Video Streaming Server
- **Port 9090**: ROS WebSocket Bridge (Rosbridge)
- **Port 3306**: MySQL Database (internal only)

### Environment Variables

#### Database Configuration
```bash
DB_HOST=db
DB_USER=rosapp  
DB_PASS=...
DB_NAME=ros_logs
```

#### Robot Configuration
```bash
ROBOT_CONTAINER=tbt3_robot
TURTLEBOT3_MODEL=waffle
```

## 📊 Database Schema

```sql
CREATE TABLE logs (
  id INT AUTO_INCREMENT PRIMARY KEY,
  ts TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  action VARCHAR(100),
  state VARCHAR(10) DEFAULT 'INFO',
  status VARCHAR(20) DEFAULT 'completed',
  details TEXT
);
```

## 🎮 Usage Guide

### Robot Control
1. **Start Robot**: Click "Start Robot" button or use API endpoint
2. **Movement Control**:
   - **Buttons**: Click directional arrows (▲▼◀▶)
   - **Keyboard**: Use WASD or arrow keys (Shift for speed boost)
   - **Joystick**: Drag the virtual joystick for analog control
3. **Stop Robot**: Click "Stop Robot" or press any stop control

### Camera System
- **Live Stream**: Automatic video feed from `/camera/image_raw` topic
- **Snapshot**: Click 📸 button to capture and save current frame
- **Stream Status**: Monitor connection via status indicators

### System Monitoring
- **Status Badge**: Real-time connection status (running/stopped/error)
- **Logs Panel**: View last 60 minutes of system activity
- **Auto-Refresh**: Logs update every 15 seconds automatically
- **Telemetry**: Live odometry data (robot position)

## 🔌 API Endpoints

### Robot Management
```bash
# Check robot health
GET /api/robot/health
# Response: {"running": true/false}

# Start/Stop robot
POST /api/toggle?action=start
POST /api/toggle?action=stop
# Response: {"status": "starting/stopping"}
```

### Logging System
```bash
# Add log entry  
POST /api/log
# Body: {"event": "string", "detail": "string", "level": "INFO/WARN/ERROR"}

# Get recent logs
GET /api/logs?minutes=60
# Response: [{"id", "ts", "level", "event", "detail"}, ...]
```

### Snapshot System
```bash
# Save camera snapshot
POST /api/snapshot  
# Body: {"dataURL": "data:image/jpeg;base64,...", "topic": "/camera/image_raw"}
# Response: {"ok": true, "file": "/snaps/snap_timestamp.jpg"}
```

## 🐛 Troubleshooting

### Common Issues

#### 1. Robot Container Not Starting
```bash
# Check container logs
docker logs tbt3_robot

# Restart robot service
docker compose restart robot
```

#### 2. WebSocket Connection Failed
```bash
# Verify rosbridge is running
docker exec -it tbt3_robot bash -c "source /opt/ros/noetic/setup.bash && rostopic list"

# Check port 9090 availability
curl -I http://localhost:9090
```

#### 3. Camera Stream Not Loading
```bash
# Check video server
curl -I "http://localhost:8080/stream?topic=/camera/image_raw"

# Verify camera topic
docker exec -it tbt3_robot bash -c "source /opt/ros/noetic/setup.bash && rostopic hz /camera/image_raw"
```

#### 4. Database Connection Issues
```bash
# Test database connectivity
docker exec -it tbt3_db mysql -urosapp -pDpt-311011 -e "SELECT 1"

# Check web service logs
docker logs tbt3_web
```

### Debug Commands

#### ROS System Check
```bash
# Enter robot container
docker exec -it tbt3_robot bash

# Check ROS topics
source /opt/ros/noetic/setup.bash
rostopic list

# Monitor cmd_vel messages
rostopic echo /cmd_vel

# Check rosbridge status
rosnode list | grep rosbridge
```

#### Web Service Debug
```bash
# Check API endpoints
curl http://localhost/api/robot/health
curl -X POST http://localhost/api/log -H 'Content-Type: application/json' -d '{"event":"test","detail":"debug"}'

# View web logs
docker logs -f tbt3_web
```

## 🔒 Security Considerations

### Current Limitations
- **No Authentication**: Open access to robot controls
- **Docker Socket Exposure**: Root access to host Docker daemon  
- **CORS Permissive**: Cross-origin requests allowed
- **Plain Text Credentials**: Database passwords in environment variables

### Production Recommendations
1. **Implement JWT Authentication**: User access control
2. **Add Rate Limiting**: Prevent abuse of control endpoints
3. **Use Docker Secrets**: Secure credential management
4. **Enable HTTPS/WSS**: Encrypted communication
5. **Network Segmentation**: Isolate robot network
6. **Input Validation**: Sanitize all user inputs

## 📈 Performance Optimization

### Latency Considerations
- **WebSocket Connection**: ~10-50ms (network dependent)
- **ROS Message Processing**: ~1-5ms (system dependent)
- **Video Streaming**: ~100-200ms (encoding/decoding)
- **Database Logging**: ~5-10ms (connection pooling)

### Scalability Improvements
- **Load Balancing**: Multiple web service instances
- **CDN Integration**: Static asset delivery
- **Database Optimization**: Indexed queries and partitioning
- **Caching Layer**: Redis for session management

### Automated Testing
```bash
# Health check script
curl -f http://localhost/api/robot/health || exit 1
curl -f http://localhost:8080/stream?topic=/camera/image_raw || exit 1

# Performance test
docker stats --no-stream
```

## 📁 Project Structure

```
itbdelabo-ros-web-integration-challenge/
├── docker-compose.yml          # Container orchestration
├── README.md                   # Project documentation
├── LICENSE                     # Project license
├── kumpulan-link.md           # Additional resources
├── db/
│   └── init.sql               # Database initialization
├── robot/
│   ├── Dockerfile             # Robot container image
│   ├── entrypoint.sh          # Robot startup script
│   ├── sim_stack.launch       # ROS launch file
│   └── launch/
│       └── turtlebot3_camera.launch
└── web/
    ├── Dockerfile             # Web service container
    ├── package.json           # Node.js dependencies
    ├── server.js              # Express.js backend
    └── index.html             # Web interface
```

## 🚀 Deployment

### Local Development
```bash
docker compose up -d --build
```

### Cloud Deployment (Azure/AWS/GCP)
1. **VM Setup**: Ubuntu 20.04+ with Docker installed
2. **Firewall**: Open ports 80, 8080, 9090
3. **Domain Setup**: Configure DNS for public access
4. **SSL Certificate**: Use Let's Encrypt for HTTPS
5. **Monitoring**: Add health checks and alerts

### Environment Variables for Production
```bash
export NODE_ENV=production
export DB_HOST=your-db-host
export DB_USER=your-db-user  
export DB_PASS=your-secure-password
export DOMAIN=your-domain.com
```

## 🤝 Contributing

### Development Setup
```bash
# Clone repository
git clone https://github.com/dietrichtarigan/itbdelabo-ros-web-integration-challenge.git
cd itbdelabo-ros-web-integration-challenge

# Start development environment
docker compose up -d --build

# Watch logs
docker compose logs -f web robot
```

