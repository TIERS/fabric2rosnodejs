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
const { buildCCPOrg2, buildWallet } = require('../javascript/AppUtil.js');
const org2 = 'Org2MSP';
const Org2UserId = 'User3';

// Text formatting
const RED = '\x1b[31m\n';
const GREENNL = '\x1b[32m\n';
const GREEN = '\x1b[32m';
const BLUE = '\x1b[34m';
const RESET = '\x1b[0m';

/**
 * Perform a sleep -- asynchronous wait
 * @param ms the time in milliseconds to sleep for
 */
function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function initGatewayForOrg2(useCommitEvents) {
  console.log(`${GREEN}--> Fabric client user & Gateway init: Using Org2 identity to Org2 Peer${RESET}`);
  // build an in memory object with the network configuration (also known as a connection profile)
  const ccpOrg2 = buildCCPOrg2();

  // build an instance of the fabric ca services client based on
  // the information in the network configuration
  const caOrg2Client = buildCAClient(FabricCAServices, ccpOrg2, 'ca.org2.example.com');

  // setup the wallet to cache the credentials of the application user, on the app server locally
  const walletPathOrg2 = path.join(__dirname, 'wallet', 'org2');
  const walletOrg2 = await buildWallet(Wallets, walletPathOrg2);

  // in a real application this would be done on an administrative flow, and only once
  // stores admin identity in local wallet, if needed
  await enrollAdmin(caOrg2Client, walletOrg2, org2);
  // register & enroll application user with CA, which is used as client identify to make chaincode calls
  // and stores app user identity in local wallet
  // In a real application this would be done only when a new user was required to be added
  // and would be part of an administrative flow
  await registerAndEnrollUser(caOrg2Client, walletOrg2, org2, Org2UserId, 'org2.department1');

  try {
    // Create a new gateway for connecting to Org's peer node.
    const gatewayOrg2 = new Gateway();

    if (useCommitEvents) {
      await gatewayOrg2.connect(ccpOrg2, {
        wallet: walletOrg2,
        identity: Org2UserId,
        discovery: { enabled: true, asLocalhost: false }
      });
    } else {
      await gatewayOrg2.connect(ccpOrg2, {
        wallet: walletOrg2,
        identity: Org2UserId,
        discovery: { enabled: true, asLocalhost: false },
        eventHandlerOptions: EventStrategies.NONE
      });
    }


    return gatewayOrg2;
  } catch (error) {
    console.error(`Error in connecting to gateway for Org2: ${error}`);
    process.exit(1);
  }
}

function printGreen(textToPrint) {
  console.log(`${GREEN} --> ` + textToPrint + `${RESET}`);
}

async function main() {

  console.log(`${BLUE} **** START ****${RESET}`);

  let contract1Org2;

  try {
    // Fabric gateway setup
    const gateway1Org2 = await initGatewayForOrg2(true);
    // Connect to Fabric network channel
    const network1Org2 = await gateway1Org2.getNetwork(channelName);
    // Connect to chaincode
    contract1Org2 = network1Org2.getContract(chaincodeName);
  } catch {
    console.error(`Error in setup: ${error}`);
    if (error.stack) {
      console.error(error.stack);
    }
    process.exit(1);
  }

  let transaction;
  // let node;
  rclnodejs.init().then(() => {
    
    printGreen(`Creating new ROS 2 node`);
    const node = new rclnodejs.Node('ros_subscriber_fabric_asset_creator');

    printGreen(`Subscribing to /ori_odom /cmd_vel ...`);

    const msg_type1 = 'nav_msgs/msg/Odometry'
    const topic1 = "/ori_odom"
    node.createSubscription(msg_type1, topic1, (msg) => {
      // var msg_string = JSON.stringify(msg)
      // console.log(`${GREEN} --> Received ROS message: ${msg_string}${RESET}`);

      try {
        // Create new asset
        console.log(`${GREEN} --> Submitting Fabric transaction: ${topic1}${RESET}`);
        transaction = contract1Org2.createTransaction('set');

        transaction.submit('/ori_odom', `{"topic": "${topic1}", "msg_type": "${msg_type1}", "msg": ${JSON.stringify(msg)}}`);
        console.log(`{"topic": "${topic1}", "msg_type": "${msg_type1}", "msg": ${JSON.stringify(msg)}}`);
        console.log(`${GREENNL} <-- Submit CreateAsset Result: committed${RESET}`);
      } catch (createError) {
        console.log(`${RED} <-- Submit Failed: CreateAsset - ${createError}${RESET}`);
      }
    });

    const msg_type2 = 'geometry_msgs/msg/Twist'
    const topic2 = "/cmd_vel"
    node.createSubscription(msg_type2, topic2, (msg) => {
      // var msg_string = JSON.stringify(msg)
      // console.log(`${GREEN} --> Received ROS message: ${msg_string}${RESET}`);

      try {
        // Create new asset
        console.log(`${GREEN} --> Submitting Fabric transaction: ${topic2}${RESET}`);
        transaction = contract1Org2.createTransaction('set');

        transaction.submit('/cmd_vel', `{"topic": "${topic2}", "msg_type": "${msg_type2}", "msg": ${JSON.stringify(msg)}}`);
        console.log(`{"topic": "${topic2}", "msg_type": "${msg_type2}", "msg": ${JSON.stringify(msg)}}`);
        console.log(`${GREENNL} <-- Submit CreateAsset Result: committed${RESET}`);
      } catch (createError) {
        console.log(`${RED} <-- Submit Failed: CreateAsset - ${createError}${RESET}`);
      }
    });

    printGreen("Spinning node...")
    node.spin();

  });

  await sleep(50000);
  console.log(`${BLUE} **** END ****${RESET}`);
  process.exit(0);
}

main();
