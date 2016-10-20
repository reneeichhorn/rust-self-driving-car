/*use piston::window::WindowSettings;
use piston::event_loop::*;
use piston::input::*;*/
/*use opengl_graphics::{ GlGraphics, OpenGL };
use piston_window::PistonWindow;
use piston_window::Glyphs;
*/
use piston_window::*;
use map::Map;
use simulation::Simulation;

const BACKGROUND: [f32; 4] = [0.1484375, 0.1953125, 0.21875, 1.0];
//const FOREGROUND: [f32; 4] = [0.921875, 0.93359375, 0.94140625, 1.0];
const CAR: [f32; 4] = [0.0, 0.475, 0.42, 1.0];
const CAR_VECTOR: [f32; 4] = [0.753, 0.792, 0.2, 1.0];
const CAR_SENSOR_INACTIVE: [f32; 4] = [0.753, 0.792, 0.2, 0.3];
const CAR_SENSOR_ACTIVE: [f32; 4] = [0.847, 0.263, 0.082, 0.3];
const CAR_METER_BG: [f32; 4] = [0.216, 0.278, 0.31, 1.0];
const CAR_METER_ACC_FG: [f32; 4] = [0.333, 0.545, 0.184, 1.0];
const CAR_METER_BRK_FG: [f32; 4] = [0.847, 0.263, 0.082, 1.0];

const ROAD_BG: [f32; 4] = [0.316, 0.378, 0.41, 1.0];

/// Application entry for graphics engine
pub struct App<'a> {
    pub simulation: Simulation,
    pub map: &'a Map
}

impl<'a> App<'a> {
    pub fn new(map: &'a Map, sim: Simulation) -> App {
        App {
            simulation: sim,
            map: map,
        }
    }

    pub fn render(&mut self, glyphs: &mut Glyphs, window: &PistonWindow, args: &RenderArgs) {
        let size = window.size();

        window.draw_2d(|mut c, gl| {
            // Clear the screen.
            clear(BACKGROUND, gl);

            // Camera
            let ui = c;
            c = c.trans(-self.simulation.car_x + 400.0, -self.simulation.car_y + 500.0);

            // car
            let car_transform = c.transform
                .trans(self.simulation.car_x, self.simulation.car_y)
                .rot_rad(self.simulation.headed);

            rectangle(CAR, [0.0, 0.0, 200.0, 100.0], car_transform, gl);

            // vector
            let line_transform = car_transform
                .trans(200.0, 50.0)
                .rot_rad(self.simulation.steering)
                .scale(100.0, 1.0);

            line(CAR_VECTOR, 2.0, [0.0, 0.0, 1.0, 0.0], line_transform, gl);

            // render sensor lines
            for sensor in &self.simulation.sensors {
                let sensor_transform = car_transform
                    .trans(sensor.x, sensor.y)
                    .rot_rad(sensor.direction)
                    .scale(sensor.range, 1.0);

                let sensor_color = if sensor.range < sensor.max_range {
                    CAR_SENSOR_ACTIVE
                } else {
                    CAR_SENSOR_INACTIVE
                };

                line(sensor_color, 2.0, [0.0, 0.0, 1.0, 0.0], sensor_transform, gl);
            }

            // render map
            for l in self.map.iter() {
                line(ROAD_BG, 4.0, [l.x1, l.y1, l.x2, l.y2], c.transform, gl);
            }

            // speedometer
            rectangle(CAR_METER_BG, [10.0, size.height as f64 - (200.0 + 10.0), 50.0, 200.0], ui.transform, gl);
            rectangle(CAR_METER_ACC_FG,
                [10.0, size.height as f64 - (200.0 + 10.0 - ((1.0 - self.simulation.get_throttle()) * 200.0)),
                 50.0, self.simulation.get_throttle() * 200.0], ui.transform, gl);

            // brakemeter
            rectangle(CAR_METER_BG, [70.0, size.height as f64 - (200.0 + 10.0), 50.0, 200.0], ui.transform, gl);
            rectangle(CAR_METER_BRK_FG,
                [70.0, size.height as f64 - (200.0 + 10.0 - ((1.0 - self.simulation.get_brake()) * 200.0)),
                 50.0, self.simulation.get_brake() * 200.0], ui.transform, gl);

        });
    }

    pub fn update(&mut self, args: &UpdateArgs) { }
}
