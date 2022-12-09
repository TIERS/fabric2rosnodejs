/*
 * Copyright TIERS Lab. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

'use strict';

// Use this to set logging, must be set before the require('fabric-network');
process.env.HFC_LOGGING = '{"debug": "./debug.log"}';

// Import Fabric libraries
const { Gateway, Wallets } = require('fabric-network');
const EventStrategies = require('fabric-network/lib/impl/event/defaulteventhandlerstrategies');
const FabricCAServices = require('fabric-ca-client');
const path = require('path');

// Import ROS
const rclnodejs = require('rclnodejs');

// Fabric network settings
const channelName = 'mychannel';
const chaincodeName = 'simplecc';

// Peer settings
const { buildCAClient, registerAndEnrollUser, enrollAdmin } = require('../javascript/CAUtil.js');
const { buildCCPOrg1, buildWallet } = require('../javascript/AppUtil.js');
const org1 = 'Org1MSP';
const Org1UserId = 'User1';

// Text formatting
const RED = '\x1b[31m\n';
const GREENNL = '\x1b[32m\n';
const GREEN = '\x1b[32m';
const BLUE = '\x1b[34m';
const RESET = '\x1b[0m';

// ROS Settings
const topics_to_publish = ["/ori_odom","/cmd_vel"];

/**
 * Perform a sleep -- asynchronous wait
 * @param ms the time in milliseconds to sleep for
 */
function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function initGatewayForOrg1(useCommitEvents) {
  console.log(`${GREEN}--> Fabric client user & Gateway init: Using Org1 identity to Org1 Peer${RESET}`);
  // build an in memory object with the network configuration (also known as a connection profile)
  const ccpOrg1 = buildCCPOrg1();

  // build an instance of the fabric ca services client based on
  // the information in the network configuration
  const caOrg1Client = buildCAClient(FabricCAServices, ccpOrg1, 'ca.org1.example.com');

  // setup the wallet to cache the credentials of the application user, on the app server locally
  const walletPathOrg1 = path.join(__dirname, 'wallet', 'org1');
  const walletOrg1 = await buildWallet(Wallets, walletPathOrg1);

  // in a real application this would be done on an administrative flow, and only once
  // stores admin identity in local wallet, if needed
  await enrollAdmin(caOrg1Client, walletOrg1, org1);
  // register & enroll application user with CA, which is used as client identify to make chaincode calls
  // and stores app user identity in local wallet
  // In a real application this would be done only when a new user was required to be added
  // and would be part of an administrative flow
  await registerAndEnrollUser(caOrg1Client, walletOrg1, org1, Org1UserId, 'org1.department1');

  try {
    // Create a new gateway for connecting to Org's peer node.
    const gatewayOrg1 = new Gateway();

    if (useCommitEvents) {
      await gatewayOrg1.connect(ccpOrg1, {
        wallet: walletOrg1,
        identity: Org1UserId,
        discovery: { enabled: true, asLocalhost: false }
      });
    } else {
      await gatewayOrg1.connect(ccpOrg1, {
        wallet: walletOrg1,
        identity: Org1UserId,
        discovery: { enabled: true, asLocalhost: false },
        eventHandlerOptions: EventStrategies.NONE
      });
    }


    return gatewayOrg1;
  } catch (error) {
    console.error(`Error in connecting to gateway for Org1: ${error}`);
    process.exit(1);
  }
}

function printGreen(textToPrint) {
  console.log(`${GREEN} --> ` + textToPrint + `${RESET}`);
}

async function main() {

  console.log(`${BLUE} **** START ****${RESET}`);

  let contract1Org1;

  try {
    // Fabric gateway setup
    const gateway1Org1 = await initGatewayForOrg1(true);
    // Connect to Fabric network channel
    const network1Org1 = await gateway1Org1.getNetwork(channelName);
    // Connect to chaincode
    contract1Org1 = network1Org1.getContract(chaincodeName);
  } catch {
    console.error(`Error in setup: ${error}`);
    if (error.stack) {
      console.error(error.stack);
    }
    process.exit(1);
  }

  var counter = 0;

  let transaction;
  // let node;
  rclnodejs.init().then(() => {
    
    printGreen(`Creating new ROS 2 node`);
    const node = new rclnodejs.Node('ros_subscriber_fabric_asset_creator');

    // printGreen(`Creating publisher to /new_odom ...`);
    var publishers = {};
    // const publisher = node.createPublisher('geometry_msgs/msg/PoseStamped', '/new_odom');

    let listener;
    try {
      // Create Fabric event listener
      listener = async (event) => {
        // Get event payload
        const asset = JSON.parse(event.payload.toString());
        printGreen("Event received !!!")
        // console.log(`${GREEN}<-- Contract Event Received: ${event.eventName} - ${JSON.stringify(asset)}${RESET}`);
        if (asset.EventName == "set") {

          // Get payload data
          console.log(asset.EventContent);
          var event_content = JSON.parse(JSON.stringify(asset.EventContent));
          
          // Publish data to ROS
          printGreen(`Getting ROS data (topic, msg type and msg) ...`);
          let ros_data;
          try {
            ros_data = JSON.parse(event_content);
          } catch (e) {
            console.log(`${RED}<-- Failed in JSON Parse - ${e}${RESET}`);
          }

          // Check if we already have a publisher or not
          let publisher;
          try {
            if (!topics_to_publish.includes(ros_data.topic)) {
              console.log(`${RED} --> Topic ${ros_data.topic} being ignored...`);
            } else {
              if ( !(ros_data.topic  in publishers) ) {
                console.log(`${GREEN} --> Creating new publisher for topic ${ros_data.topic} and type  ${ros_data.msg_type}.${RESET}`);
                publishers[ros_data.topic] = node.createPublisher(ros_data.msg_type, ros_data.topic);
                publisher = publishers[ros_data.topic];
              } else {
                publisher = publishers[ros_data.topic];
              }
              // console.log(ros_data.msg);
              printGreen("Publishing data to ROS...")
              if (ros_data.msg_type == "nav_msgs/msg/Odometry") {
                console.log(`${BLUE}  --> Got Odometry data. Deleting covariance matrix...`);
                try {
                  delete ros_data.msg.pose.covariance;
                  delete ros_data.msg.twist.covariance; 
                  // delete ros_data.msg.pose.covariance;
                } catch(e) {
                  console.log(`${RED}<-- Failed cleaning Odometry msg..... - ${e}${RESET}`);
                }
                try {
                  console.log(++counter);
                  // var new_msg = {
                  //   header: {
                  //   stamp: {
                  //       sec: 123,
                  //       nanosec: 100,
                  //   },
                  //   frame_id: "world"
                  //   },
                  //   child_frame_id: "base_footprint",
                  //   pose: {
                  //     pose: {
                  //         position: {
                  //             x: 0.5,
                  //             y: 0.0,
                  //             z: 0.0
                  //         },
                  //         orientation: {
                  //             x: 0.0,
                  //             y: 0.0,
                  //             z: 0.0,
                  //             w: 1.0
                  //         }
                  //     }
                  //   },
                    
                  //   twist: {
                  //     twist: {
                  //         linear: {
                  //             x: 1.0,
                  //             y: 1.0,
                  //             z: 0.0
                  //         },
                  //         angular: {
                  //             x: 0.0,
                  //             y: 0.0,
                  //             z: 0.0,
                  //         }
                  //     }
                  //   }
                  // };
                  var new_msg = ros_data.msg;
                  // console.log(new_msg);
                  // console.log(new_msg.pose.pose.position);
                  // console.log(publisher);
                  publisher.publish(new_msg);
                } catch(e) {
                  console.log(`${RED}<-- Failed publishing Odometry msg to ROS - ${e}${RESET}`);
                }
              } else {
                try {
                  console.log(++counter);
                  // console.log(ros_data.msg)
                  publisher.publish(ros_data.msg);
                } catch(e) {
                  console.log(`${RED}<-- Failed publishing msg to ROS - ${e}${RESET}`);
                }
              }
            }
          } catch(e) {
            console.log(`${RED}<-- Failed getting ROS publisher - ${e}${RESET}`);
          }

        } else {
          console.log(`${RED}<-- Event other than 'set' received !!!`)
        }
      };
      // now start the client side event service and register the listener
      console.log(`${GREEN} --> Start contract event stream to peer in Org1${RESET}`);
      contract1Org1.addContractListener(listener);
    } catch (eventError) {
      console.log(`${RED} <-- Failed: Setup contract events - ${eventError}${RESET}`);
      process.exit(1);
    }

    printGreen("Spinning node...\n\n")
    node.spin();

  });

  await sleep(50000);
  console.log(`${BLUE} **** END ****${RESET}`);
  process.exit(0);
}

main();
