const rclnodejs = require('rclnodejs');

// function doPub(publisher) {
//   publisher.publish(msg);
// }

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('geometry_msgs/msg/PoseStamped', '/odom');

  msg = {
      header: {
      stamp: {
          sec: 123,
          nanosec: 100,
      },
      frame_id: "world"
      },
      pose: {
          position: {
              x: 0.5,
              y: 0.0,
              z: 0.0
          },
          orientation: {
              x: 0.0,
              y: 0.0,
              z: 0.0,
              w: 1.0
          }
      }
  };

  var count = 0;

  setInterval(function() {

    count += 0.5;
    msg.pose.position.x = count;
    publisher.publish(msg);
  }, 20);

});
