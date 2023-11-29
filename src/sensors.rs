use std::sync::{Arc, Mutex};

use futures::executor::LocalSpawner;
use futures::future;
use futures::StreamExt;
use futures::task::LocalSpawnExt;
use nalgebra::{Quaternion, UnitQuaternion};
use r2r::geometry_msgs::msg::{Point, Pose, Twist, Vector3};
use r2r::qos::ReliabilityPolicy;
use r2r::QosProfile;
use r2r::sensor_msgs::msg::{Imu, LaserScan, Range};
use r2r::std_msgs::msg::Int32 as RosI32;
use r2r::std_msgs::msg::String as RosString;

pub struct RangeData {
    pub ranges: [f32; 4],
}

pub struct ImuData {
    // Roll, Pitch, Yaw from -PI to PI
    pub quaternion: UnitQuaternion<f64>,
    // z+ is CCW rotation from above
    pub rotation: Vector3,
}

impl RangeData {
    pub fn new(spawner: &LocalSpawner, node: &mut r2r::Node, topic: &str) -> Arc<Mutex<Self>> {
        let data = Arc::new(Mutex::new(Self {
            ranges: [10.0; 4],
        }));

        let names = ["fr", "fl", "rl", "rr"];
        for i in 0..4 {
            let subscriber = node.subscribe::<Range>(&*(topic.to_owned() + "/" + names[i]), QosProfile::default().reliability(ReliabilityPolicy::BestEffort)).unwrap();
            let data = data.clone();
            spawner.spawn_local(async move {
                subscriber.for_each(|msg| {
                    let mut lock = data.lock().unwrap();
                    lock.ranges[i] = msg.range;
                    future::ready(())
                }).await
            }).unwrap();
        }
        data
    }
}

impl ImuData {
    pub fn new(spawner: &LocalSpawner, node: &mut r2r::Node, topic: &str) -> Arc<Mutex<Self>> {
        let data = Arc::new(Mutex::new(Self {
            quaternion: UnitQuaternion::default(),
            rotation: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        }));

        let subscriber = node.subscribe::<Imu>(topic, QosProfile::default()).unwrap();
        let data_clone = data.clone();
        spawner.spawn_local(async move {
            subscriber.for_each(|msg| {
                let mut lock = data.lock().unwrap();
                lock.quaternion = UnitQuaternion::from_quaternion(Quaternion::new(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z));
                lock.rotation = msg.angular_velocity;
                future::ready(())
            }).await
        }).unwrap();
        data_clone
    }
}
