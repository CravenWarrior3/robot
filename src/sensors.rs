use std::sync::{Arc, Mutex};

use futures::executor::LocalSpawner;
use futures::future;
use futures::StreamExt;
use futures::task::LocalSpawnExt;
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Twist, Vector3};
use r2r::QosProfile;
use r2r::sensor_msgs::msg::{LaserScan, Range};
use r2r::std_msgs::msg::Int32 as RosI32;
use r2r::std_msgs::msg::String as RosString;
use r2r::unitysim_msgs::msg::BoundingBox3d;

pub struct TrackerData {
    pub position: f64,
    pub size: f64,
    pub valid: bool,
}

pub struct RangeData {
    pub ranges: [f32; 4],
}

impl TrackerData {
    pub fn new(spawner: &LocalSpawner, node: &mut r2r::Node, topic: &str) -> Arc<Mutex<Self>> {
        //subscriber: impl Stream<Item=BoundingBox3d> + Unpin + 'static
        let subscriber = node.subscribe::<BoundingBox3d>(topic, QosProfile::default()).unwrap();
        let data = Arc::new(Mutex::new(Self {
            position: 0.0,
            size: 0.0,
            valid: false,
        }));
        let data_clone = data.clone();

        spawner.spawn_local(async move {
            subscriber.for_each(|msg| {
                let mut lock = data.lock().unwrap();
                lock.position = msg.center.position.y;
                lock.size = msg.size.x;
                lock.valid = true;
                future::ready(())
            }).await
        }).unwrap();

        data_clone
    }
}

impl RangeData {
    pub fn new(spawner: &LocalSpawner, node: &mut r2r::Node, topic: &str) -> Arc<Mutex<Self>> {
        let data = Arc::new(Mutex::new(Self {
            ranges: [10.0; 4],
        }));

        let names = ["fr", "fl", "rl", "rr"];
        for i in 0..4 {
            let subscriber = node.subscribe::<Range>(&*(topic.to_owned() + "/" + names[i]), QosProfile::default()).unwrap();
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
