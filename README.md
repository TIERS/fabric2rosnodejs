# fabric2rosnodejs for multi-host environment

Install all dependencies: Ros noetic + nodejs + tello ROS driver
Node.js v16.x Installation:
```bash
curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash - &&\
sudo apt-get install -y nodejs
```
Install nodejs library for rclnodejs:
```bash
npm install rclnodejs
```

Configure network hosts in all the hosts
```bash
sudo nano /etc/hosts
192.168.xx.xx orderer.example.com 
192.168.xx.xx peer0.org1.example.com
192.168.xx.xx ca.org1.example.com
192.168.xx.xx peer0.org2.example.com
192.168.xx.xx ca.org2.example.com
192.168.xx.xx peer0.org3.example.com
192.168.xx.xx ca.org3.example.com
```

On all hosts, clone the repository
```bash
git clone git@github.com:TIERS/fabric2rosnodejs.git
```

On each host, bring up the network with script hostXup.sh
```bash
./hostXup.sh
```

On the orderer host, run the script to bring up the channel,  join all peers to the channel, and deploy the chaincode
```bash
./mychannelup.sh
```

On the corresponding host, go to the `application-javascript-ros-hostX` folder and run both publisher and subscriber
```bash
node app-ros-publisher.js
# (another terminal)
node app-ros-subscriber.js
```

Now you should be able to see ros topics in the hosts！😊
