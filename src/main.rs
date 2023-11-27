use std::f64::consts::PI as PI;

use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r::QosProfile;
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Twist, Vector3};
use r2r::sensor_msgs::msg::{Range, LaserScan};
use r2r::std_msgs::msg::Int32 as RosI32;
use r2r::std_msgs::msg::String as RosString;
use r2r::unitysim_msgs::msg::BoundingBox3d;
use crate::sensors::{ImuData, RangeData};

mod sensors;

const MAX_TURN: f64 = 7.33;

#[derive(Eq, PartialEq, Debug)]
enum MoveMode {
    FollowLeft,
    FollowRight,
    Wander,
}

struct ArcStats {
    min: f32,
    max: f32,
    average: f32,
}

impl ArcStats {
    fn from_index(start: usize, end: usize, laser: &LaserScan) -> Self {
        let mut stats = Self {
            min: laser.range_max,
            max: laser.range_min,
            average: 0.0,
        };

        let count = end - start;
        for i in start..end {
            let mut range = laser.ranges[i];

            // Check valid reading
            if range > laser.range_max {
                range = 1.5 * laser.range_max;
            } else if range < laser.range_min {
                range = laser.range_min;
            }

            // Update statistics
            if range > stats.max {
                stats.max = range;
            }
            if range < stats.min {
                stats.min = range;
            }
            stats.average += range;
        }
        stats.average /= count as f32;

        stats
    }

    fn from_degrees(start: f32, end: f32, laser: &LaserScan) -> Self {
        let start = angle_to_index(start.to_radians(), &laser);
        let end = angle_to_index(end.to_radians(), &laser);

        let mut stats = Self {
            min: laser.range_max,
            max: laser.range_min,
            average: 0.0,
        };

        let count = end - start;
        for i in start..end {
            let mut range = laser.ranges[i];

            // Check valid reading
            if range > laser.range_max {
                range = 1.5 * laser.range_max;
            } else if range < laser.range_min {
                range = laser.range_min;
            }

            // Update statistics
            if range > stats.max {
                stats.max = range;
            }
            if range < stats.min {
                stats.min = range;
            }
            stats.average += range;
        }
        stats.average /= count as f32;

        stats
    }
}

fn to_changes(laser: &LaserScan) -> Vec<f32> {
    let mut changes = Vec::new();

    for i in 0..(laser.ranges.len() - 1) {
        changes.push(laser.ranges[i + 1] - laser.ranges[i]);
    }

    changes
}

fn index_to_angle(index: usize, laser: &LaserScan) -> f32 {
    laser.angle_min + (index as f32 * laser.angle_increment)
}

fn angle_to_index(angle: f32, laser: &LaserScan) -> usize {
    ((angle - laser.angle_min) / laser.angle_increment) as usize
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "robot", "")?;

    // Primary sensors
    let laser = node.subscribe::<LaserScan>("/scan", QosProfile::default())?;
    let velocity = node.create_publisher::<Twist>("/cmd_vel", QosProfile::default())?;

    // Secondary sensors
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    let rangefinder = RangeData::new(&spawner, &mut node, "/range");
    let imu = ImuData::new(&spawner, &mut node, "/imu_broadcaster/imu");

    // Laser scanner drives mobility
    {
        let mut velocity_cache = Twist {
            linear: Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };

        let mut movement_mode = MoveMode::Wander;
        let (mut cooldown, mut repeat) = (0, 0);

        spawner.spawn_local(async move {
            laser.for_each(|scan| {
                // Repeated actions and movement mode reset
                if repeat > 0 {
                    repeat -= 1;
                    velocity.publish(&velocity_cache).unwrap();
                    return future::ready(());
                } else if cooldown == 0 {
                    cooldown = 60;
                    movement_mode = MoveMode::Wander;
                    println!("MOVEMENT RESET\n");
                }

                let mut msg = Twist {
                    linear: Vector3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                };

                // Operate laser scanner
                let left = ArcStats::from_degrees(-90.0, -75.0, &scan);
                let front_left = ArcStats::from_degrees(-175.0, -172.5, &scan);
                let front = {
                    let mut fl = ArcStats::from_degrees(-180.0, -172.5, &scan);
                    let fr = ArcStats::from_degrees(172.5, 180.0, &scan);

                    fl.average += fr.average;
                    fl.average /= 2.0;
                    if fr.min < fl.min {
                        fl.min = fr.min;
                    }
                    if fr.max > fl.max {
                        fl.max = fr.max;
                    }

                    fl
                };
                let front_right = ArcStats::from_degrees(172.5, 175.0, &scan);
                let right = ArcStats::from_degrees(75.0, 90.0, &scan);
                let left_hemi = ArcStats::from_degrees(-170.0, 0.0, &scan);
                let right_hemi = ArcStats::from_degrees(0.0, 170.0, &scan);

                // Choose turn direction and speed
                msg.angular.z = match movement_mode {
                    MoveMode::FollowLeft => ((left_hemi.min - 0.5) * 0.75) as f64,
                    MoveMode::FollowRight => ((0.5 - right_hemi.min) * 0.75) as f64,
                    MoveMode::Wander => {
                        // Switch movement mode
                        // TODO: Implement better wall follow
                        /*if cooldown > 40 && cooldown < 50 && (left.average - right.average).abs() > 1.0 {
                            if left.average < 4.0 {
                                movement_mode = MoveMode::FollowLeft;
                            }
                            if right.average < 4.0 {
                                movement_mode = MoveMode::FollowRight;
                            }
                        }*/
                        ((right_hemi.average - left_hemi.average) * 0.3) as f64
                    }
                };

                let imu = imu.lock().unwrap();
                let imu_angles = imu.quaternion.euler_angles();
                //println!("X: {}, Y: {}, Z: {}", imu_angles.0, imu_angles.1, imu_angles.2);

                // Our laser scanner doesn't work up close, check the rangefinders too
                let rangefinder = rangefinder.lock().unwrap();
                let front_min = front.min.min(rangefinder.ranges[0].min(rangefinder.ranges[1]));

                // Forward speed reduction
                if front_min < 1.5 {
                    msg.linear.x = front.min as f64 / 5.0;
                }
                // Turn speed cap
                if msg.angular.z > MAX_TURN {
                    msg.angular.z = MAX_TURN;
                } else if msg.angular.z < -MAX_TURN {
                    msg.angular.z = -MAX_TURN;
                }
                // Avoid hitting walls
                if front_min < 0.25 {
                    // TODO: Try using the IMU data for rotations
                    println!("STUCK\n");
                    msg.angular.z = MAX_TURN;
                    repeat = 3;
                    velocity_cache = msg.clone();
                } else if front_min < 0.5 {
                    msg.angular.z += if rangefinder.ranges[0] < rangefinder.ranges[1] {
                        -MAX_TURN
                    } else {
                        MAX_TURN
                    }
                }

                cooldown -= 1;

                velocity.publish(&msg).unwrap();
                future::ready(())
            }).await
        })?;
    }

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
