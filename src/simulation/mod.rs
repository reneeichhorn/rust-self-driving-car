use std::vec::*;

use ncollide::shape::*;
use ncollide::ray::*;
use ncollide::bounding_volume::*;
use ncollide::broad_phase::*;
use nalgebra::*;

use map::Map;

#[derive(Clone)]
pub struct DistanceSensor {
    pub x: f64,
    pub y: f64,
    pub direction: f64,
    pub max_range: f64,
    pub range: f64,
}

#[derive(Clone)]
pub struct Vec2D {
    pub x: f64,
    pub y: f64
}

impl Vec2D {
    pub fn len(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

#[derive(Clone)]
pub struct Simulation {
    acceleration: Vec2D,
    local_acceleration: Vec2D,
    yaw_rate: f64,

    throttle: f64,
    brake: f64,
    pub steering: f64,

    pub headed: f64,
    velocity: Vec2D,
    pub local_velocity: Vec2D,

    pub car_x: f64,
    pub car_y: f64,
    pub map_collision: Polyline<Pnt2<f64>>,
    pub sensors: Vec<DistanceSensor>
}

// Physic constants
const CAR_MAX_STEERING: f64 = 0.02;
const GRAVITY: f64 = 9.81;
const CAR_MASS: f64 = 1200.0;
const CAR_ENGINE_FORCE: f64 = 8000.0;
const CAR_BRAKE_FORCE: f64 = 12000.0;
const CAR_INTERTIA: f64 = CAR_MASS * 1.0;
const CAR_GRAVITY_HEIGHT: f64 = 0.55;
const CAR_GRAVITY_AXLE_FRONT: f64 = 1.25;
const CAR_GRAVITY_AXLE_REAR: f64 = 1.25;
const CAR_WHEEL_BASE: f64 = CAR_GRAVITY_AXLE_FRONT + CAR_GRAVITY_AXLE_REAR;
const CAR_WEIGHT_TRANSFER: f64 = 0.2;
const CAR_WEIGHT_FRONT: f64 = CAR_GRAVITY_AXLE_REAR / CAR_WHEEL_BASE;
const CAR_WEIGHT_REAR: f64 = CAR_GRAVITY_AXLE_FRONT / CAR_WHEEL_BASE;
const CAR_TIRE_GRIP: f64 = 2.0;
const CAR_STIFFNESS_FRONT: f64 = 5.0;
const CAR_STIFFNESS_REAR: f64 = 5.2;
const CAR_ROLL_RESIST: f64 = 8.0;
const CAR_AIR_RESIST: f64 = 2.5;

fn sign(val: f64) -> f64 {
    if val > 0.0 {
        1.0
    } else if val < 0.0 {
        -1.0
    } else {
        0.0
    }
}

fn mclamp(n: f64, min: f64, max: f64) -> f64 {
    n.max(min).min(max)
}

impl Simulation {
    pub fn new (map: &Map, map_collision: Polyline<Pnt2<f64>>) -> Self {
        Simulation {
            throttle: 0.0,
            brake: 0.0,
            acceleration: Vec2D { x: 0.0, y: 0.0 },
            local_acceleration: Vec2D { x: 0.0, y: 0.0 },
            yaw_rate: 0.0,
            steering: 0.0,
            headed: -1.57079633,
            car_x: map.start_x,//1196.0,
            car_y: map.start_y,//908.0,
            map_collision: map_collision,
            velocity: Vec2D {x: 0.0, y: 0.0},
            local_velocity: Vec2D {x: 0.0, y: 0.0},
            sensors: vec![
                // Upper left
                DistanceSensor { x: 10.0, y: 0.0, direction: -2.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 0.0, direction: -2.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 0.0, direction: -1.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 0.0, direction: -1.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 0.0, direction: -0.57079633, max_range: 200.0, range: 200.0 },
                // Lower left
                DistanceSensor { x: 10.0, y: 100.0, direction: 2.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 100.0, direction: 2.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 100.0, direction: 1.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 100.0, direction: 1.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 10.0, y: 100.0, direction: 0.57079633, max_range: 200.0, range: 200.0 },
                // Upper right
                DistanceSensor { x: 190.0, y: 0.0, direction: -2.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 0.0, direction: -2.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 0.0, direction: -1.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 0.0, direction: -1.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 0.0, direction: -0.57079633, max_range: 200.0, range: 200.0 },
                // Lower right
                DistanceSensor { x: 190.0, y: 100.0, direction: 2.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 100.0, direction: 2.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 100.0, direction: 1.57079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 100.0, direction: 1.07079633, max_range: 200.0, range: 200.0 },
                DistanceSensor { x: 190.0, y: 100.0, direction: 0.57079633, max_range: 200.0, range: 200.0 },
                // Front
                DistanceSensor { x: 200.0, y: 50.0, direction: -0.5, max_range: 300.0, range: 300.0 },
                DistanceSensor { x: 200.0, y: 50.0, direction: -0.25, max_range: 300.0, range: 300.0 },
                DistanceSensor { x: 200.0, y: 50.0, direction: 0.0, max_range: 300.0, range: 300.0 },
                DistanceSensor { x: 200.0, y: 50.0, direction: 0.25, max_range: 300.0, range: 300.0 },
                DistanceSensor { x: 200.0, y: 50.0, direction: 0.5, max_range: 300.0, range: 300.0 },
            ]
         }
    }

    pub fn reset(&mut self, map:& Map) {
        self.throttle = 0.0;
        self.brake = 0.0;
        self.acceleration = Vec2D { x: 0.0, y: 0.0 };
        self.local_acceleration = Vec2D { x: 0.0, y: 0.0 };
        self.yaw_rate = 0.0;
        self.steering = 0.0;
        self.headed = -1.57079633;
        self.car_x = map.start_x;
        self.car_y = map.start_y;
        self.velocity = Vec2D {x: 0.0, y: 0.0};
        self.local_velocity = Vec2D {x: 0.0, y: 0.0};
    }

    pub fn update_colission(&mut self, map_collision: Polyline<Pnt2<f64>>) {
        self.map_collision = map_collision;
    }

    pub fn set_acceleration_input(&mut self, input: f64) {
        if input >= 0.0 {
            self.throttle = input;
            self.brake = 0.0;
        } else {
            self.throttle = 0.0;
            self.brake = -1.0 * input;
        }
    }

    pub fn get_throttle(&self) -> f64 {
        self.throttle
    }

    pub fn get_brake(&self) -> f64 {
        self.brake
    }

    pub fn get_sensor_ray(&self, i: usize) -> Ray<Pnt2<f64>> {
        // rotate coordinates
        let rot = Rot2::new(Vec1::new(self.headed));
        let pos = rotate(&rot, &Pnt2::new(self.sensors[i].x, self.sensors[i].y));

        Ray::new(
            Pnt2::new(pos.x + self.car_x, pos.y + self.car_y),
            Vec2::new(
                (self.sensors[i].direction + self.headed).cos(),
                (self.sensors[i].direction + self.headed).sin()))
    }

    pub fn tick (&mut self, elapsed: f64) {
        let sn = self.headed.sin();
        let cs = self.headed.cos();
        let steer = self.steering * CAR_MAX_STEERING;

        // get local velocity
        self.local_velocity = Vec2D {
            x: cs * self.velocity.x + sn * self.velocity.y,
            y: cs * self.velocity.y - sn * self.velocity.x
        };

        // get weights on axis'
        let weightFront: f64 = CAR_MASS *
            (CAR_WEIGHT_FRONT * GRAVITY - CAR_WEIGHT_TRANSFER * self.local_acceleration.x * CAR_GRAVITY_HEIGHT / CAR_WHEEL_BASE);
        let weightRear: f64 = CAR_MASS *
            (CAR_WEIGHT_REAR * GRAVITY + CAR_WEIGHT_TRANSFER * self.local_acceleration.x * CAR_GRAVITY_HEIGHT / CAR_WHEEL_BASE);

        // velocity based on yaw rate
        let yawSpeedFront = CAR_GRAVITY_AXLE_FRONT * self.yaw_rate;
        let yawSpeedRear = -CAR_GRAVITY_AXLE_REAR * self.yaw_rate;

        // get alpha/slip angle
        let slipAngleFront = (self.local_velocity.y + yawSpeedFront).atan2(self.local_velocity.x.abs()) - sign(self.local_velocity.x) * steer;
        let slipAngleRear = (self.local_velocity.y + yawSpeedRear).atan2(self.local_velocity.x.abs());

        // get tire grip
        let tireGripFront = CAR_TIRE_GRIP;
        let tireGripRear = CAR_TIRE_GRIP;

        // get friction
        let frictionForceFront = mclamp(-CAR_STIFFNESS_FRONT * slipAngleFront, -tireGripFront, tireGripFront) * weightFront;
        let frictionForceRear = mclamp(-CAR_STIFFNESS_REAR * slipAngleRear, -tireGripRear, tireGripRear) * weightRear;

        // get amont of braking and throtlling
        let brake = (self.brake * CAR_BRAKE_FORCE).min(CAR_BRAKE_FORCE);
        let throttle = self.throttle * CAR_ENGINE_FORCE;

        // get resulting force
        let tractionForce = Vec2D {
            x: throttle - brake * sign(self.local_velocity.x),
            y: 0.0
        };

        // get resulting drag force
        let dragForce = Vec2D {
            x: -CAR_ROLL_RESIST * self.local_velocity.x - CAR_AIR_RESIST * self.local_velocity.x * self.local_velocity.x.abs(),
            y: -CAR_ROLL_RESIST * self.local_velocity.y - CAR_AIR_RESIST * self.local_velocity.y * self.local_velocity.y.abs(),
        };

        // total force
        let totalForce = Vec2D {
            x: dragForce.x + tractionForce.x,
            y: dragForce.y + tractionForce.y + steer.cos() * frictionForceFront + frictionForceRear
        };

        // add acceleration
        self.local_acceleration = Vec2D { x: totalForce.x  / CAR_MASS, y: totalForce.y / CAR_MASS };
        self.acceleration = Vec2D {
            x: cs * self.local_acceleration.x - sn * self.local_acceleration.y,
            y: sn * self.local_acceleration.x + cs * self.local_acceleration.y,
        };

        // add velocity
        self.velocity.x += self.acceleration.x * elapsed;
        self.velocity.y += self.acceleration.y * elapsed;

        // rotational force
        let mut absoluteVelocity = self.velocity.len();
        let mut angularTorque = (frictionForceFront + tractionForce.y) * CAR_GRAVITY_AXLE_FRONT - frictionForceRear * CAR_GRAVITY_AXLE_REAR;

        // check collision with map
        let mut car_stopped = false;
        /*let car_rect = Cuboid::new(Vec2::new(100.0, 50.0));
        let car_aabb = aabb(&car_rect, &Iso2::new(Vec2::new(self.car_x + 100.0, self.car_y), Vec1::new(self.headed)));
        let mut car_stopped = false;

        let mut index = 0;
        for map_aabb in self.map_collision.bounding_volumes() {
            if map_aabb.intersects(&car_aabb) {
                car_stopped = true;
                println!("true {}", index)
                //break;
            } else {
                println!("false {}", index)
            }
            index += 1;
        }*/

        // check sensor collisions
        for i in 0..self.sensors.len() {
            self.sensors[i].range = self.sensors[i].max_range;

            let ray = self.get_sensor_ray(i);
            let res = self.map_collision.toi_with_ray(&Identity::new(), &ray, false);

            match res {
                Some(toi) => {
                    if toi < 2.0 {
                        car_stopped = true;
                    }
                    if toi < self.sensors[i].max_range {
                        self.sensors[i].range = toi;
                    }
                },
                None => {
                }
            }
        }

        // stop the car when it's to slow and no force is given
        if (absoluteVelocity.abs() < 0.5 && throttle <= 0.0) || car_stopped {
            self.velocity = Vec2D { x: 0.0, y: 0.0 };
            self.yaw_rate = 0.0;
            angularTorque = 0.0;
            absoluteVelocity = 0.0;
        }

        // update rotation
        let angularAccelaration = angularTorque / CAR_INTERTIA;
        self.yaw_rate += angularAccelaration * elapsed;
        self.headed += self.yaw_rate * elapsed;

        // update position
        self.car_x += self.velocity.x * elapsed;
        self.car_y += self.velocity.y * elapsed;

    }
}
