use std::f64::consts::PI as PI;

use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use pid::Pid;
use r2r::QosProfile;
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Twist, Vector3};
use r2r::sensor_msgs::msg::{Range, LaserScan};
use r2r::std_msgs::msg::Int32 as RosI32;
use r2r::std_msgs::msg::String as RosString;
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
        let mut movement_mode = MoveMode::Wander;
        let (mut cooldown, mut decision_lockout) = (0, 0);
        let mut target_angle = 0.0;
        let mut follow_angle = false;
        let mut pid = Pid::new(0.0, MAX_TURN);
        pid.p(0.2, MAX_TURN).i(0.2, MAX_TURN).d(0.4, MAX_TURN);

        spawner.spawn_local(async move {
            laser.for_each(|scan| {
                // Movement mode reset
                if cooldown == 0 {
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
                        ((right_hemi.average - left_hemi.average) * 0.1) as f64
                    }
                };

                let imu = imu.lock().unwrap();
                let imu_angles = imu.quaternion.euler_angles();

                // Our laser scanner doesn't work up close, check the rangefinders too
                let rangefinder = rangefinder.lock().unwrap();
                let front_min = front.min.min(rangefinder.ranges[0].min(rangefinder.ranges[1]));

                // Speed control
                if imu_angles.1.abs() > 0.2 {
                    // Stuck on wall, reverse
                    msg.linear.x = -0.25;
                } else if front_min < 1.0 {
                    // Approaching wall, slow down
                    msg.linear.x = front.min as f64 / 4.0;
                }

                // Turn speed cap
                if msg.angular.z > MAX_TURN {
                    msg.angular.z = MAX_TURN;
                } else if msg.angular.z < -MAX_TURN {
                    msg.angular.z = -MAX_TURN;
                }
                // Avoid hitting walls
                if front_min < 0.4 && decision_lockout < 0 {
                    println!("U-TURN\n");
                    target_angle = imu_angles.2 + if imu_angles.2 >= 0.0 {
                        -PI
                    } else {
                        PI
                    };
                    pid.reset_integral_term();
                    follow_angle = true;
                    decision_lockout = 120;
                } else if front_min < 0.75 {
                    msg.angular.z += if rangefinder.ranges[0] < rangefinder.ranges[1] {
                        -MAX_TURN
                    } else {
                        MAX_TURN
                    }
                }

                if follow_angle {
                    let mut turn_angle = (target_angle - imu_angles.2).sin().atan2((target_angle - imu_angles.2).cos());
                    let output = pid.next_control_output(turn_angle);

                    msg.linear.x = 0.0;
                    msg.angular.z = output.output;
                    if turn_angle.abs() < 0.3 {
                        follow_angle = false;
                        msg.angular.z = 0.0;
                        println!("MANEUVER END\n");
                    }
                }

                cooldown -= 1;
                decision_lockout -= 1;

                velocity.publish(&msg).unwrap();
                future::ready(())
            }).await
        })?;
    }

    loop {
        node.spin_once(std::time::Duration::from_millis(10));
        pool.run_until_stalled();
    }
}
